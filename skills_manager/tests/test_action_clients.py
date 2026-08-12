import inspect

from skills_manager import home, play_skill, record_skill


def test_legacy_clients_do_not_construct_a_robot_owner():
    source = "\n".join(inspect.getsource(module) for module in (
        home, play_skill, record_skill,
    ))
    assert "SpinPandaNode" not in source
    assert "LfD(" not in source


def test_play_skill_builds_canonical_command_from_archive_name():
    command = play_skill.command_from_skill_name("touch__cityboard")

    assert command.action == "touch"
    assert command.objects == ["cityboard"]
    assert command.command == "touch cityboard"
