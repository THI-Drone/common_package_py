import pytest
import time
import json
import rclpy

from common_package_py.commands import JsonKeyDefinition, data_type, CommandDefinitions


def test_command_definition():
    # Test the definition of the "waypoint" command
    with pytest.raises(RuntimeError):
        def1 = CommandDefinitions.get_definition("waypoint")

    # Test the definition of the "detect_marker" command
    def1 = CommandDefinitions.get_definition("detect_marker")
    def2 = CommandDefinitions.get_detect_marker_command_definition()
    assert def1 == def2

    # Test the definition of the "drop_payload" command
    with pytest.raises(RuntimeError):
        def1 = CommandDefinitions.get_definition("drop_payload")

    # Test the definition of the "end_mission" command
    def1 = CommandDefinitions.get_definition("end_mission")
    def2 = dict()
    assert def1 == def2

    # Test the definition of an invalid command
    with pytest.raises(RuntimeError):
        def1 = CommandDefinitions.get_definition("abc")


def test_parse_check_json_str():
    # Checking that string parsing works

    # Check invalid json string
    string = "invalid json"
    definition = {"key": JsonKeyDefinition(True, data_type.STRING)}

    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json_str(string, definition)

    # Check valid json string
    string = "{\"key\":\"valid json\"}"
    definition = {"key": JsonKeyDefinition(True, data_type.STRING)}

    CommandDefinitions.parse_check_json_str(string, definition)


def test_parse_check_json():
    # Key checks

    # Check that all keys are present and valid
    json_obj = {"null": None,
                "boolean": True,
                "number": 123,
                "number_float": 1.234,
                "number_integer": -5,
                "number_unsigned": 3,
                "string": "abc",
                "array": [1, 2, 3, 4, 5],
                "object": {"one": 1, "two": 2}}

    definition = {"null": JsonKeyDefinition(True, data_type.NULL),
                  "boolean": JsonKeyDefinition(True, data_type.BOOLEAN),
                  "number": JsonKeyDefinition(True, data_type.NUMBER),
                  "number_float": JsonKeyDefinition(True, data_type.NUMBER_FLOAT),
                  "number_integer": JsonKeyDefinition(True, data_type.NUMBER_INTEGER),
                  "number_unsigned": JsonKeyDefinition(True, data_type.NUMBER_UNSIGNED),
                  "string": JsonKeyDefinition(True, data_type.STRING),
                  "array": JsonKeyDefinition(True, data_type.ARRAY),
                  "object": JsonKeyDefinition(True, data_type.OBJECT)}

    CommandDefinitions.parse_check_json(json_obj, definition)

    # Check required and optional keys
    definition = {"null": JsonKeyDefinition(True, data_type.NULL),
                  "boolean": JsonKeyDefinition(False, data_type.BOOLEAN),
                  "number": JsonKeyDefinition(True, data_type.NUMBER),
                  "number_float": JsonKeyDefinition(False, data_type.NUMBER_FLOAT),
                  "number_integer": JsonKeyDefinition(True, data_type.NUMBER_INTEGER),
                  "number_unsigned": JsonKeyDefinition(False, data_type.NUMBER_UNSIGNED),
                  "string": JsonKeyDefinition(True, data_type.STRING),
                  "array": JsonKeyDefinition(False, data_type.ARRAY),
                  "object": JsonKeyDefinition(True, data_type.OBJECT)}

    # All optional keys present
    json_obj = {"null": None,
                "boolean": True,
                "number": 123,
                "number_float": 1.234,
                "number_integer": -5,
                "number_unsigned": 3,
                "string": "abc",
                "array": [1, 2, 3, 4, 5],
                "object": {"one": 1, "two": 2}}

    CommandDefinitions.parse_check_json(json_obj, definition)

    # No optional keys present
    json_obj = {"null": None,
                "number": 123,
                "number_integer": -5,
                "string": "abc",
                "object": {"one": 1, "two": 2}}

    CommandDefinitions.parse_check_json(json_obj, definition)

    # One optional key present
    json_obj = {"null": None,
                "boolean": True,
                "number": 123,
                "number_integer": -5,
                "string": "abc",
                "object": {"one": 1, "two": 2}}

    CommandDefinitions.parse_check_json(json_obj, definition)

    # One required key missing
    json_obj = {"null": None,
                "boolean": True,
                "number_float": 1.234,
                "number_integer": -5,
                "number_unsigned": 3,
                "string": "abc",
                "array": [1, 2, 3, 4, 5],
                "object": {"one": 1, "two": 2}}

    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # One unknown key
    json_obj = {"null": None,
                "boolean": True,
                "unknown_key": "this shouldn't be here",
                "number": 123,
                "number_float": 1.234,
                "number_integer": -5,
                "number_unsigned": 3,
                "string": "abc",
                "array": [1, 2, 3, 4, 5],
                "object": {"one": 1, "two": 2}}

    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # All keys missing
    json_obj = dict()

    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Value checks: Check that value is in bounds and of the correct type

    # Define the key and its corresponding definition
    definition = {"number_unsigned": JsonKeyDefinition(
        True, data_type.NUMBER_UNSIGNED, 3, 6)}

    # Test case: value is within the specified bounds
    json_obj = {"number_unsigned": 3}
    CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is below the specified lower bound
    json_obj = {"number_unsigned": 2}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is not of the correct type
    json_obj = {"number_unsigned": 3.0}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is above the specified upper bound
    json_obj = {"number_unsigned": 7}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Define the key and its corresponding definition
    definition = {"number_unsigned": JsonKeyDefinition(
        True, data_type.NUMBER_UNSIGNED, max_val=6)}

    # Test case: value is within the specified bounds
    json_obj = {"number_unsigned": 3}
    CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is within the specified bounds
    json_obj = {"number_unsigned": 2}
    CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is not of the correct type
    json_obj = {"number_unsigned": 3.0}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is above the specified upper bound
    json_obj = {"number_unsigned": 7}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Define the key and its corresponding definition
    definition = {"number_unsigned": JsonKeyDefinition(
        True, data_type.NUMBER_UNSIGNED, min_val=3)}

    # Test case: value is within the specified bounds
    json_obj = {"number_unsigned": 3}
    CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is below the specified lower bound
    json_obj = {"number_unsigned": 2}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is not of the correct type
    json_obj = {"number_unsigned": 3.0}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is within the specified bounds
    json_obj = {"number_unsigned": 7}
    CommandDefinitions.parse_check_json(json_obj, definition)

    # Define the key and its corresponding definition
    definition = {"number_unsigned": JsonKeyDefinition(
        True, {data_type.NUMBER_UNSIGNED, data_type.NUMBER_INTEGER})}

    # Test case: value is within the specified bounds
    json_obj = {"number_unsigned": 3}
    CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is not of the correct type
    json_obj = {"number_unsigned": -2.123}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is not of the correct type
    json_obj = {"number_unsigned": 3.0}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is within the specified bounds
    json_obj = {"number_unsigned": 7}
    CommandDefinitions.parse_check_json(json_obj, definition)

    # Define the key and its corresponding definition
    definition = {"number_unsigned": JsonKeyDefinition(
        True, {data_type.NUMBER_UNSIGNED, data_type.NUMBER_INTEGER}, 3, 6)}

    # Test case: value is within the specified bounds
    json_obj = {"number_unsigned": 3}
    CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is not of the correct type
    json_obj = {"number_unsigned": -2.123}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is not of the correct type
    json_obj = {"number_unsigned": 3.0}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)

    # Test case: value is above the specified upper bound
    json_obj = {"number_unsigned": 7}
    with pytest.raises(RuntimeError):
        CommandDefinitions.parse_check_json(json_obj, definition)


if __name__ == "__main__":
    pytest.main()
