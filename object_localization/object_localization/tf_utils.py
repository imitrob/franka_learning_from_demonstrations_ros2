import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class CustomTransformListener():
    """TF lookups backed by tf2_ros.

    The previous version kept its own {(parent, child): TransformStamped} dict
    fed from /tf and /tf_static, and matched only a single *directly broadcast*
    edge. That happened to work here -- panda.py broadcasts panda_link0 ->
    panda_hand directly and static_transform_camera.py broadcasts panda_hand ->
    camera_color_optical_frame -- but it always returned the newest message
    regardless of the stamp asked for, so a lookup silently used stale
    extrinsics while the arm moved, and it broke outright for any frame pair
    reached through more than one hop.

    tf2's buffer walks the chain, inverts edges as needed, and interpolates to
    a requested stamp.
    """

    def __init__(self):
        super(CustomTransformListener, self).__init__()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def lookup_relative_transform(self, source_frame, target_frame, at_time=None,
                                  future_tolerance=None):
        """Translation+rotation of target_frame expressed in source_frame.

        That is T_source<-target, the same thing the old dict returned for the
        key (source_frame, target_frame). Returns (None, None) when the
        transform is not available.

        Pass at_time (a rclpy.time.Time, e.g. built from an image
        header.stamp) to get the transform as it was at that instant rather
        than the latest one -- that is what keeps a lookup honest while the arm
        is moving.

        future_tolerance (a rclpy.duration.Duration) makes a stamp slightly
        *newer* than the freshest sample fall back to that sample instead of
        failing. This is not pedantry: a broadcaster running at N Hz leaves a
        1/N second hole after each sample, and an image whose pipeline latency is
        shorter than that hole gets stamped inside it. tf2 will not extrapolate
        forward, so the lookup fails even though the arm's pose is perfectly well
        known to within a few milliseconds. Bounded, because the whole point of
        asking at a stamp is to not use a transform from somewhere else.
        """
        requested = Time() if at_time is None else at_time
        try:
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                requested,
                # ponytail: zero timeout on purpose. SpinningRosNode runs a
                # SingleThreadedExecutor, so blocking here from inside a
                # service callback would deadlock the very executor that fills
                # the buffer. Callers already retry; give them a fast miss.
                timeout=Duration(seconds=0),
            )
        except tf2_ros.ExtrapolationException as error:
            if at_time is None or future_tolerance is None:
                self.get_logger().warning(
                    f"Transform {source_frame} -> {target_frame} not available: {error}")
                return None, None
            return self._latest_within(
                source_frame, target_frame, at_time, future_tolerance, error)
        except tf2_ros.TransformException as error:
            self.get_logger().warning(
                f"Transform {source_frame} -> {target_frame} not available: {error}"
            )
            return None, None
        return transform.transform.translation, transform.transform.rotation

    def _latest_within(self, source_frame, target_frame, at_time, future_tolerance, error):
        """The newest transform, if it is no more than future_tolerance older."""
        try:
            transform = self.tf_buffer.lookup_transform(
                source_frame, target_frame, Time(), timeout=Duration(seconds=0))
        except tf2_ros.TransformException:
            self.get_logger().warning(
                f"Transform {source_frame} -> {target_frame} not available: {error}")
            return None, None

        # Nanoseconds rather than Time arithmetic: subtracting two rclpy Times
        # raises if their clock types differ, and a caller has no reason to know
        # that Time.from_msg defaults to ROS_TIME while Time() defaults to system
        # time. A units comparison cannot care.
        behind_ns = at_time.nanoseconds - Time.from_msg(transform.header.stamp).nanoseconds
        if behind_ns > future_tolerance.nanoseconds:
            self.get_logger().warning(
                f"Transform {source_frame} -> {target_frame} is "
                f"{behind_ns / 1e6:.0f} ms behind the requested stamp, more "
                f"than the {future_tolerance.nanoseconds / 1e6:.0f} ms tolerated: {error}")
            return None, None

        # Debug, not warning: at the rates involved this is the normal case, and
        # the substitution is what keeps a slow broadcaster usable.
        self.get_logger().debug(
            f"Transform {source_frame} -> {target_frame} taken "
            f"{behind_ns / 1e6:.1f} ms before the requested stamp")
        return transform.transform.translation, transform.transform.rotation


def main(args=None):
    """Smoke check: resolve one chained transform and print it."""
    import threading

    rclpy.init(args=args)

    # CustomTransformListener.__init__ calls super().__init__() with no arguments,
    # so the class after it in the MRO has to supply the node name itself --
    # otherwise this reaches Node.__init__() and raises about a missing
    # node_name. That is how LocalizationService composes it (via
    # SpinningRosNode), and it is what made this smoke check unusable before.
    class _Named(Node):
        def __init__(self):
            super().__init__("tf_utils_probe")

    class _Probe(CustomTransformListener, _Named):
        pass

    node = _Probe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    rate = node.create_rate(10)

    source_frame, target_frame = 'panda_link0', 'panda_hand'
    try:
        # the buffer needs a moment of spinning before the chain is complete
        for _ in range(50):
            translation, rotation = node.lookup_relative_transform(source_frame, target_frame)
            if translation is not None:
                node.get_logger().info(
                    f"Transform {source_frame} -> {target_frame}:\n"
                    f"Translation: x={translation.x}, y={translation.y}, z={translation.z}\n"
                    f"Rotation: x={rotation.x}, y={rotation.y}, z={rotation.z}, w={rotation.w}"
                )
                break
            rate.sleep()
        else:
            node.get_logger().error(
                f"Transform {source_frame} -> {target_frame} never became available"
            )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
