import json
import rclpy
from enum import IntEnum, unique
from common_package_py.common_node import CommonNode


@unique
class data_type(IntEnum):
    """Enumeration representing the data types."""
    NULL = 1
    BOOLEAN = 2
    NUMBER = 3
    NUMBER_FLOAT = 4
    NUMBER_INTEGER = 5
    NUMBER_UNSIGNED = 6
    STRING = 7
    ARRAY = 8
    OBJECT = 9


class JsonKeyDefinition:
    """
    Represents a definition for a JSON key.

    Args:
        required (bool): Indicates whether the key is required.
        data_types (set[data_type] | data_type): Set of data types that the key can have or a single data type.
        min_val (float | None, optional): Minimum value allowed for numeric keys. Defaults to None.
        max_val (float | None, optional): Maximum value allowed for numeric keys. Defaults to None.
    """

    def __init__(self, required: bool, data_types: set[data_type] | data_type, min_val: float | None = None, max_val: float | None = None) -> None:
        self.required = required

        # Check if data_types is a single data_type and convert it to a set
        if isinstance(data_types, data_type):
            data_types = {data_types}

        # Check if data_types is a set
        if not isinstance(data_types, set):
            raise RuntimeError(
                f"JsonKeyDefinition::JsonKeyDefinition: data_types has the wrong type: {type(data_types)}")

        # Check if each element in data_types is of type data_type
        for dt in data_types:
            if not isinstance(dt, data_type):
                raise RuntimeError(
                    f"JsonKeyDefinition::JsonKeyDefinition: data_type '{dt}' has the wrong type: {type(dt)}. Expected: {type(data_type)}")

        # Check if data_types is not empty
        if len(data_types) <= 0:
            raise RuntimeError(
                "JsonKeyDefinition::JsonKeyDefinition: data_types is empty")

        # Set the data_types attribute
        self.data_types = data_types

        # Check if min_val and max_val are provided and swap them if min_val > max_val
        if min_val is not None and max_val is not None and min_val > max_val:
            self.min_val, self.max_val = max_val, min_val
        else:
            self.min_val, self.max_val = min_val, max_val

    @staticmethod
    def data_type_to_string(data_type: data_type) -> str:
        """
        Converts a data type to its string representation.

        Args:
            data_type (data_type): The data type to convert.

        Returns:
            str: The string representation of the data type.
        """

        return data_type.name.lower()

    def check_bounds(self, json_iter: any) -> bool:
        """
        Checks if the value of the JSON key is within the specified bounds.
        The bounds are defined by the minimum and maximum values allowed for the
        supported data types. Only specified bounds are checked. If no bounds are
        specified, all values will be accepted. Every data type that is not of
        any `number*` data type will always return true.

        Note:
            Also returns true if the json_iter has a type different from `number*`

        Args:
            json_iter (any): The value of the JSON key.

        Returns:
            bool: True if the value is within the bounds, False otherwise.
        """

        jsk_type_check = JsonKeyDefinition(False, {data_type.NUMBER})

        if data_type.NUMBER in self.data_types or \
                data_type.NUMBER_FLOAT in self.data_types or \
        data_type.NUMBER_INTEGER in self.data_types or \
                data_type.NUMBER_UNSIGNED in self.data_types:

            if self.min_val is not None and jsk_type_check.type_check(json_iter) and json_iter < self.min_val:
                return False
            if self.max_val is not None and jsk_type_check.type_check(json_iter) and json_iter > self.max_val:
                return False
        return True

    def type_check(self, json_iter: any) -> bool:
        """
        Checks if the value of the JSON key matches any of the allowed data types.

        Args:
            json_iter (any): The value of the JSON key.

        Returns:
            bool: True if the value matches any of the allowed data types, False otherwise.
        """

        for data_type in self.data_types:
            match data_type:
                case data_type.NULL:
                    if json_iter is None:
                        return True
                case data_type.BOOLEAN:
                    if isinstance(json_iter, bool):
                        return True
                case data_type.NUMBER:
                    if isinstance(json_iter, (int, float)) and not isinstance(json_iter, (bool)):
                        return True
                case data_type.NUMBER_FLOAT:
                    if isinstance(json_iter, float) and not isinstance(json_iter, (bool)):
                        return True
                case data_type.NUMBER_INTEGER:
                    if isinstance(json_iter, int) and not isinstance(json_iter, (bool)):
                        return True
                case data_type.NUMBER_UNSIGNED:
                    if isinstance(json_iter, int) and json_iter >= 0 and not isinstance(json_iter, (bool)):
                        return True
                case data_type.STRING:
                    if isinstance(json_iter, str):
                        return True
                case data_type.ARRAY:
                    if isinstance(json_iter, list):
                        return True
                case data_type.OBJECT:
                    if isinstance(json_iter, dict):
                        return True
                case _:
                    raise RuntimeError(
                        f"JsonKeyDefinition::type_check: Unknown data_type provided: {data_type}")

        return False

    def __eq__(self, other: 'JsonKeyDefinition') -> bool:
        """
        Checks if two JsonKeyDefinition objects are equal.

        Args:
            other (JsonKeyDefinition): The other JsonKeyDefinition object to compare.

        Returns:
            bool: True if the objects are equal, False otherwise.
        """

        return self.required == other.required and self.data_types == other.data_types and self.min_val == other.min_val and self.max_val == other.max_val


class CommandDefinitions:
    """
    A class that provides methods for parsing and checking JSON objects based on a given definition.

    Attributes:
        None

    Methods:
        parse_check_json_str(json_str: str, definition: dict) -> json:
            Parses and checks a JSON string based on the given definition.

        parse_check_json(json_obj: json, definition: dict) -> json:
            Parses and checks a JSON object based on the given definition.
    """

    def __init__(self) -> None:
        pass

    @staticmethod
    def parse_check_json_str(json_str: str, definition: dict) -> json:
        """
        Parses and checks a JSON string based on the given definition.

        Note:
            The function can only be fully used on shallow JSONs. Arrays or
            encapsulations contents' will not be checked! Call this function several
            times with the different parts to get that behavior.

        Args:
            json_str (str): The JSON string to parse and check.
            definition (dict): The definition of the JSON object structure.

        Returns:
            json: The parsed and checked JSON object.

        Raises:
            RuntimeError: With an error message why the parsing or check failed
        """

        try:
            candidate = json.loads(json_str)
        except json.JSONDecodeError as e:
            raise RuntimeError(
                f"CommandDefinitions::parse_check_json_str: Failed to parse JSON: {str(e)}, lineno: {e.lineno}, colno: {e.colno}, pos: {e.pos}")

        return CommandDefinitions.parse_check_json(candidate, definition)

    @staticmethod
    def parse_check_json(json_obj: json, definition: dict) -> json:
        """
        Parses and checks a JSON object based on the given definition.

        Only use this version if you already have a json object.
        If you only have a string, use the `parse_check_json_str` function
        instead to do a proper parsing.

        Steps:
        - Checks that no undefined keys are in the JSON
        - Checks that all required keys exist
        - Checks that all values have the correct type

        Note:
            The function can only be fully used on shallow JSONs. Arrays or
            encapsulations contents' will not be checked! Call this function several
            times with the different parts to get that behavior.

        Args:
            json_obj (json): The JSON object to parse and check.
            definition (dict): The definition of the JSON object structure.

        Returns:
            json: The parsed and checked JSON object.

        Raises:
            RuntimeError: If unknown keys are found, if required keys are missing, if values are out of bounds,
                or if values have the wrong type. An error message why the check failed is included.
        """

        # Check that all keys are allowed
        # Set with all unknown keys that were found
        unknown_keys = json_obj.keys() - definition

        # Check if at least one unknown key was found
        if len(unknown_keys) > 0:
            raise RuntimeError(
                f"CommandDefinitions::parse_check_json: Unknown key(s) found: {', '.join(unknown_keys)}")

        # Check keys and value types
        for key, json_definition in definition.items():
            if key not in json_obj:
                if json_definition.required:  # Check that key exists if required
                    raise RuntimeError(
                        f"CommandDefinitions::parse_check_json: Missing required key: {key}")
                else:  # If key is not required and doesn't exist, continue with next key
                    continue

            search = json_obj[key]  # Current value that will be checked

            # Check if value is in bound (only supported for numbers)
            if not json_definition.check_bounds(search):
                error_msg = f"CommandDefinitions::parse_check_json: [Key: '{key}'] Value '{search}' is out of bounds: "

                # Message when min and max values are defined
                if json_definition.min_val is not None and json_definition.max_val is not None:
                    error_msg += f"[{json_definition.min_val}; {json_definition.max_val}]"
                elif json_definition.min_val is not None:  # Message when only the min value is defined
                    error_msg += f"> {json_definition.min_val}"
                elif json_definition.max_val is not None:  # Message when only the max value is defined
                    error_msg += f"< {json_definition.max_val}"

                raise RuntimeError(error_msg)

            # Check that type of value matches the definition
            type_check = json_definition.type_check(search)

            if not type_check:  # Type check failed
                # Trying to be as helpful as possible
                allowed_types = []

                for data_type in json_definition.data_types:
                    allowed_types.append(
                        JsonKeyDefinition.data_type_to_string(data_type))

                raise RuntimeError(
                    f"CommandDefinitions::parse_check_json: [Key: '{key}'] Value '{search}' is of the wrong type. Allowed type(s): {', '.join(allowed_types)}")

        # Json object successfully passed all checks
        return json_obj

    @staticmethod
    def get_detect_marker_command_definition() -> dict[str, JsonKeyDefinition]:
        """
        Returns the command definition for detecting markers.

        Returns:
            dict[str, JsonKeyDefinition]: The command definition with the following keys:
                - 'detection_height_cm': A JsonKeyDefinition object representing the detection height in centimeters.
        """

        return {"timeout_ms": JsonKeyDefinition(True, data_type.NUMBER_UNSIGNED, 0, 3 * 60 * 1000)}

    @staticmethod
    def get_definition(type: str) -> dict[str, JsonKeyDefinition]:
        """
        Retrieves the definition for the given command type.

        Args:
            type (str): The type of the command.

        Returns:
            dict[str, JsonKeyDefinition]: The command definition.

        Raises:
            RuntimeError: If the command type is not recognized or not available in the Python version.
        """

        match type:
            case "waypoint":
                raise RuntimeError(
                    f"This definition is currently not available in the Python version. Type: {type}")
            case "detect_marker":
                return CommandDefinitions.get_detect_marker_command_definition()
            case "set_marker":
                raise RuntimeError(
                    f"This definition is currently not available in the Python version. Type: {type}")
            case "end_mission":
                return dict()
            case _:
                raise RuntimeError(
                    f"CommandDefinitions::get_definition: Unknown type: {type}")
