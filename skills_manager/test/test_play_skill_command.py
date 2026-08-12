"""A part-numbered skill name has to expand into both parts on one object."""
from skills_manager.play_skill import command_from_skill_name


def test_single_part_skill_names_one_object():
    command = command_from_skill_name("pick__taskboard")
    assert command.action == "pick"
    assert command.objects == ["taskboard"]


def test_two_part_skill_repeats_the_object():
    command = command_from_skill_name("pick_and_place1__taskboard")
    assert command.action == "pick_and_place"
    assert command.objects == ["taskboard", "taskboard"]
