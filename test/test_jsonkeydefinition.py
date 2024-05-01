import pytest

from common_package_py.commands import JsonKeyDefinition, data_type


def test_json_key_definition_constructor():
    # Constructor with everything possible
    every_data_type = {
        data_type.NULL, data_type.BOOLEAN, data_type.NUMBER,
        data_type.NUMBER_FLOAT, data_type.NUMBER_INTEGER, data_type.NUMBER_UNSIGNED,
        data_type.STRING, data_type.ARRAY, data_type.OBJECT
    }
    jsk = JsonKeyDefinition(
        required=False, data_types=every_data_type, min_val=-10, max_val=10)
    assert not jsk.required
    assert jsk.data_types == every_data_type
    assert jsk.min_val == -10
    assert jsk.max_val == 10

    # Invalid constructor
    with pytest.raises(RuntimeError):
        JsonKeyDefinition(required=True, data_types=set())
    with pytest.raises(RuntimeError):
        JsonKeyDefinition(required=True, data_types="abc")
    with pytest.raises(RuntimeError):
        JsonKeyDefinition(required=True, data_types={"abc"})

    # Constructor with minimum
    data_types = data_type.NULL
    jsk = JsonKeyDefinition(required=True, data_types=data_types)
    assert jsk.required
    assert jsk.data_types == {data_types}
    assert jsk.min_val is None
    assert jsk.max_val is None

    # Constructor with min_val
    data_types = data_type.NULL
    jsk = JsonKeyDefinition(required=True, data_types=data_types, min_val=3)
    assert jsk.required
    assert jsk.data_types == {data_types}
    assert jsk.min_val == 3
    assert jsk.max_val is None

    # Constructor with max_val
    data_types = data_type.NULL
    jsk = JsonKeyDefinition(required=True, data_types=data_types, max_val=4)
    assert jsk.required
    assert jsk.data_types == {data_types}
    assert jsk.min_val is None
    assert jsk.max_val == 4

    # Constructor with min_val > max_val
    data_types = data_type.NULL
    jsk = JsonKeyDefinition(
        required=True, data_types=data_types, min_val=5, max_val=4)
    assert jsk.required
    assert jsk.data_types == {data_types}
    assert jsk.min_val == 4
    assert jsk.max_val == 5


def test_json_key_definition_type_check_and_bounds_check():
    # Test with number data type and specified min and max values
    jsk = JsonKeyDefinition(False, {data_type.NUMBER}, -10, 10)

    # Test with valid integer value within the specified range
    json_obj = 5
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid integer value at the lower bound
    json_obj = -10
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid integer value at the upper bound
    json_obj = 10
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid floating-point value within the specified range
    json_obj = 1.345
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with floating-point value outside the specified range
    json_obj = 10.00000001
    assert jsk.type_check(json_obj)
    assert not jsk.check_bounds(json_obj)

    # Test with integer value outside the specified range
    json_obj = -11
    assert jsk.type_check(json_obj)
    assert not jsk.check_bounds(json_obj)

    # Test with string value (invalid type)
    json_obj = "abc"
    assert not jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with nested JSON object (invalid type)
    json_obj = {"abc": 123}
    assert not jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with number_float and number_unsigned data types and specified min and max values
    jsk = JsonKeyDefinition(
        False, {data_type.NUMBER_FLOAT, data_type.NUMBER_UNSIGNED}, -10, 10)

    # Test with valid unsigned integer value within the specified range
    json_obj = 5
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid floating-point value within the specified range
    json_obj = 0.234
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with invalid integer value (outside the specified range)
    json_obj = -1
    assert not jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with invalid unsigned integer value (outside the specified range)
    json_obj = 11
    assert jsk.type_check(json_obj)
    assert not jsk.check_bounds(json_obj)

    # Test with invalid floating-point value (outside the specified range)
    json_obj = 10.123
    assert jsk.type_check(json_obj)
    assert not jsk.check_bounds(json_obj)

    # Test with invalid integer value (outside the specified range)
    json_obj = -11
    assert not jsk.type_check(json_obj)
    assert not jsk.check_bounds(json_obj)

    # Test with number_float data type and specified min and max values
    jsk = JsonKeyDefinition(False, {data_type.NUMBER_FLOAT}, -10, 10)

    # Test with valid floating-point value within the specified range
    json_obj = 0.1234
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid floating-point value at the upper bound
    json_obj = 10.0
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with invalid integer value
    json_obj = -1
    assert not jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with number data type and specified min value
    jsk = JsonKeyDefinition(False, {data_type.NUMBER}, -10)

    # Test with invalid string value (invalid type)
    json_obj = "abc"
    assert not jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid floating-point value
    json_obj = 1234.0
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with invalid integer value (outside the specified range)
    json_obj = -11
    assert jsk.type_check(json_obj)
    assert not jsk.check_bounds(json_obj)

    # Test with number data type and specified max value
    jsk = JsonKeyDefinition(False, {data_type.NUMBER}, max_val=10)

    # Test with invalid string value (invalid type)
    json_obj = "abc"
    assert not jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid floating-point value
    json_obj = -1234.0
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with invalid integer value (outside the specified range)
    json_obj = 11
    assert jsk.type_check(json_obj)
    assert not jsk.check_bounds(json_obj)

    # Test with number data type (no min or max value specified)
    jsk = JsonKeyDefinition(False, {data_type.NUMBER})

    # Test with invalid string value (invalid type)
    json_obj = "abc"
    assert not jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid floating-point value
    json_obj = -1234.0
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with valid integer value
    json_obj = -1234
    assert jsk.type_check(json_obj)
    assert jsk.check_bounds(json_obj)

    # Test with all possible data types
    data_types = {data_type.NULL, data_type.BOOLEAN, data_type.NUMBER, data_type.NUMBER_FLOAT,
                  data_type.NUMBER_INTEGER, data_type.NUMBER_UNSIGNED, data_type.STRING, data_type.ARRAY, data_type.OBJECT}

    for dt in data_types:
        jsk = JsonKeyDefinition(False, {dt})

        # Test with null value
        json_obj = None
        if dt == data_type.NULL:
            assert jsk.type_check(json_obj)
        else:
            assert not jsk.type_check(json_obj)

        assert jsk.check_bounds(json_obj)

        # Test with boolean value
        json_obj = True
        if dt == data_type.BOOLEAN:
            assert jsk.type_check(json_obj)
        else:
            assert not jsk.type_check(json_obj)

        assert jsk.check_bounds(json_obj)

        # Test with integer value
        json_obj = 11
        if dt in (data_type.NUMBER_INTEGER, data_type.NUMBER, data_type.NUMBER_UNSIGNED):
            assert jsk.type_check(json_obj)
        else:
            assert not jsk.type_check(json_obj)

        assert jsk.check_bounds(json_obj)

        # Test with floating-point value
        json_obj = 1.234
        if dt in (data_type.NUMBER_FLOAT, data_type.NUMBER):
            assert jsk.type_check(json_obj)
        else:
            assert not jsk.type_check(json_obj)

        assert jsk.check_bounds(json_obj)

        # Test with unsigned integer value
        json_obj = 3
        if dt in (data_type.NUMBER_UNSIGNED, data_type.NUMBER, data_type.NUMBER_INTEGER):
            assert jsk.type_check(json_obj)
        else:
            assert not jsk.type_check(json_obj)

        assert jsk.check_bounds(json_obj)

        # Test with string value
        json_obj = "abc"
        if dt == data_type.STRING:
            assert jsk.type_check(json_obj)
        else:
            assert not jsk.type_check(json_obj)

        assert jsk.check_bounds(json_obj)

        # Test with array value
        json_obj = [1, 2, 3, 8, 16]
        if dt == data_type.ARRAY:
            assert jsk.type_check(json_obj)
        else:
            assert not jsk.type_check(json_obj)

        assert jsk.check_bounds(json_obj)

        # Test with object value
        json_obj = {"key1": "value1", "key2": "value2"}
        if dt == data_type.OBJECT:
            assert jsk.type_check(json_obj)
        else:
            assert not jsk.type_check(json_obj)

        assert jsk.check_bounds(json_obj)


def test_json_key_definition_data_type_to_string():
    # Test that the data_type_to_string function returns the correct string representation for each valid data type
    assert JsonKeyDefinition.data_type_to_string(data_type.NULL) == "null"
    assert JsonKeyDefinition.data_type_to_string(
        data_type.BOOLEAN) == "boolean"
    assert JsonKeyDefinition.data_type_to_string(data_type.NUMBER) == "number"
    assert JsonKeyDefinition.data_type_to_string(
        data_type.NUMBER_FLOAT) == "number_float"
    assert JsonKeyDefinition.data_type_to_string(
        data_type.NUMBER_INTEGER) == "number_integer"
    assert JsonKeyDefinition.data_type_to_string(
        data_type.NUMBER_UNSIGNED) == "number_unsigned"
    assert JsonKeyDefinition.data_type_to_string(data_type.STRING) == "string"
    assert JsonKeyDefinition.data_type_to_string(data_type.ARRAY) == "array"
    assert JsonKeyDefinition.data_type_to_string(data_type.OBJECT) == "object"


if __name__ == "__main__":
    pytest.main()
