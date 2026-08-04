from typing import NamedTuple, Optional

import numpy as np
import cv2
MIN_MATCH_COUNT = 8


class Match(NamedTuple):
    """One template located in the current image. Pure image space, no frames.

    u, v     pixel of the object in the *current* image: the template's
             crop-box centre mapped through the fitted affine.
    scale    s = Z_template / Z_now. Greater than 1 means the object appears
             larger than in the template, i.e. closer to the camera.
    yaw      rotation in image coordinates (radians), RELATIVE to the template
             capture orientation. That reference is recorded nowhere, so this is
             "how far it turned since the template was taken", not a heading.
    inliers  points accepted by estimateAffinePartial2D's RANSAC.
    """
    u: float
    v: float
    scale: float
    yaw: float
    inliers: int


def detect_scene_features(image):
    """SIFT keypoints and descriptors for one frame, shareable across templates.

    get_scene matches every saved template against the same picture, and
    detecting on that picture costs ~133 ms of the ~160 ms each template used to
    take. Detecting once per frame instead of once per template is what lets the
    scene be published at a useful rate with more than one template saved.

    A fresh detector per call rather than a shared one: SIFT_create() costs about
    a microsecond, while sharing a cv2.SIFT across the reentrant callback group
    LocalizationService uses would be a data race for no gain.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.SIFT_create().detectAndCompute(gray, None)


class Localizer(object):

    def __init__(self, template, cropping, depth):
        assert isinstance(template, str)
        self._full_template = cv2.imread(template, 0)
        self._box_depth = depth
        cropped_h = cropping[2:]
        cropped_w = cropping[:2]

        self._template = self._full_template[
            cropped_h[0] : cropped_h[1], cropped_w[0] : cropped_w[1]
        ]
        self.delta_translation = np.array([
            cropped_w[0],
            cropped_h[0],
        ], dtype=np.float32)
        # Centre of the crop box, in FULL-template coordinates. You drew that box
        # around the object when capturing the template, so its centre is the
        # object centre by construction -- unlike a keypoint centroid, which
        # drifts toward whichever side happens to be textured.
        self._crop_centre = np.array([
            0.5 * (cropped_w[0] + cropped_w[1]),
            0.5 * (cropped_h[0] + cropped_h[1]),
        ], dtype=np.float32)

        self._sift = cv2.SIFT_create()
        self._flann = cv2.FlannBasedMatcher(
            dict(algorithm=0, trees=5),  # FLANN_INDEX_KDTREE
            dict(checks=100),
        )
        # The template image never changes, so detect on it once per instance
        # instead of once per frame. Keypoints are shifted into full-template
        # coordinates here, so detect_points must not shift them again.
        self._kp_template, self._des_template = self._sift.detectAndCompute(self._template, None)
        for keypoint in self._kp_template:
            keypoint.pt = (
                keypoint.pt[0] + self.delta_translation[0],
                keypoint.pt[1] + self.delta_translation[1],
            )

        self._src_pts = None
        self._dst_pts = None
        self._annoted_image = None

    def set_image(self, img) -> None:
        self._img = img

    def set_camera_info(self, msg):
        self.cx_cy_array = np.array([[msg.k[2]], [msg.k[5]]])
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._pixel_m_factor_u =  self._fx / self._box_depth
        self._pixel_m_factor_v =  self._fy / self._box_depth

    def detect_points(self, scene_features=None, annotate=True):
        """Match this template into the current image.

        scene_features -- (keypoints, descriptors) from detect_scene_features()
            for the current frame. Pass them when several templates share one
            picture; leave as None to detect here, which is what the servo path
            does and what this method has always done.
        annotate -- build the /SIFT_localization debug image. Costs ~20 ms per
            template and only the servo path ever reads it.
        """
        # Clear last frame's result first. Instances are reused now, so leaving
        # stale _src_pts behind would let a template that matched once keep
        # reporting that old position forever.
        self._src_pts = None
        self._dst_pts = None
        self._annoted_image = None

        kp1, des1 = self._kp_template, self._des_template
        if scene_features is None:
            kp2, des2 = self._sift.detectAndCompute(
                cv2.cvtColor(self._img, cv2.COLOR_BGR2GRAY), None)
        else:
            kp2, des2 = scene_features
        if des1 is None or des2 is None or len(des2) < 2:
            print("WARNING: LOCALIZER NOT FOUND TEMPLATE!", flush=True)
            return

        # find matches by knn which calculates point distance in 128 dim
        matches = self._flann.knnMatch(des1, des2, k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good.append(m)

        if len(good) > MIN_MATCH_COUNT:
            self._src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            self._dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        else:
            print("WARNING: LOCALIZER NOT FOUND TEMPLATE!", flush=True)
            return

        if not annotate:
            return

        try:
            M, mask = cv2.findHomography(self._src_pts, self._dst_pts, cv2.RANSAC, 5.0)
            matchesMask = mask.ravel().tolist()
            draw_params = dict(
                matchColor=(0, 255, 0),
                singlePointColor=None,
                matchesMask=matchesMask,
                flags=2,
            )
            self._annoted_image = cv2.drawMatches(self._full_template, kp1, self._img, kp2, good, None, **draw_params)

        except Exception as e:
            print("error orccured", e, flush=True)

    def annoted_image(self):
        return self._annoted_image

    def measure_object(self) -> Optional[Match]:
        """Locate the object in the current image. Call after detect_points().

        Returns None when there is nothing to measure. Deliberately separate
        from compute_full_tf_in_m: that one builds a *servo delta* from
        principal-point-centred coordinates, which destroys the absolute
        translation term needed here, and discards the affine's scale.
        """
        if self._src_pts is None or self._dst_pts is None:
            return None

        affine, inlier_mask = cv2.estimateAffinePartial2D(self._src_pts, self._dst_pts)
        if affine is None:
            return None

        linear = affine[0:2, 0:2]
        # estimateAffinePartial2D returns [s*R | t] -- 4 DOF. det(s*R) = s^2.
        scale = float(np.sqrt(abs(np.linalg.det(linear))))
        if scale <= 0.0:
            return None

        centre = linear @ self._crop_centre + affine[0:2, 2]
        return Match(
            u=float(centre[0]),
            v=float(centre[1]),
            scale=scale,
            yaw=float(np.arctan2(affine[1, 0], affine[0, 0])),
            inliers=int(inlier_mask.sum()) if inlier_mask is not None else 0,
        )

    def template_range(self) -> float:
        """Camera-to-object range assumed when the template was captured (m)."""
        return float(self._box_depth)

    def compute_tf(self) -> np.ndarray:

        p0 = np.transpose(np.array(self._src_pts))[:, 0, :] - self.cx_cy_array
        p1 = np.transpose(np.array(self._dst_pts))[:, 0, :] - self.cx_cy_array
        T0 = compute_transform(p0, p1) #this matrix is a 3x3

        return T0

    def compute_full_tf_in_m(self) -> np.ndarray:
        T0 = self.compute_tf()
        T0[0, 2] /= self._pixel_m_factor_u
        T0[1, 2] /= self._pixel_m_factor_v
        T = np.identity(4)
        T[0:2, 0:2] = T0[0:2, 0:2]
        T[0:2, 3] = T0[0:2, 2]
        return T

def compute_transform(points: np.ndarray, transformed_points: np.ndarray):
    assert isinstance(points, np.ndarray)
    assert isinstance(transformed_points, np.ndarray)
    assert points.shape == transformed_points.shape
    cv2_matrix, _ = cv2.estimateAffinePartial2D(points.T, transformed_points.T)
    return cv2_matrix
