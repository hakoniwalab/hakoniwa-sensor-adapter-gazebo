#!/usr/bin/env python3
"""Convert Gazebo SDF sensor definitions to and from Hakoniwa JSON."""

import argparse
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


HOST_TO_GAZEBO = {
    "imu": "imu",
    "mag": "magnetometer",
    "baro": "air_pressure",
    "gps": "navsat",
    "force_torque": "force_torque",
}
GAZEBO_TO_HOST = {
    gazebo_type: host_type
    for host_type, gazebo_type in HOST_TO_GAZEBO.items()
}

INTEGER_RE = re.compile(r"^[+-]?\d+$")
FLOAT_CHARACTERS_RE = re.compile(r"^[0-9+\-.eE]+$")


class ConversionError(ValueError):
    """Raised when an input cannot be represented by the other format."""


def _convert_text(text):
    """Convert stripped XML text according to the SDF-to-JSON rules."""
    value = text.strip()
    if value == "":
        return ""

    if INTEGER_RE.fullmatch(value):
        return int(value)

    if FLOAT_CHARACTERS_RE.fullmatch(value):
        try:
            number = float(value)
        except ValueError:
            pass
        else:
            if math.isfinite(number):
                return number

    return value


def _element_to_data(element):
    children = list(element)
    if not element.attrib and not children:
        return _convert_text(element.text or "")

    result = {}
    for name, value in element.attrib.items():
        result["@" + name] = value

    if element.text and element.text.strip():
        result["#text"] = _convert_text(element.text)

    for child in children:
        name = child.tag
        value = _element_to_data(child)
        if name not in result:
            result[name] = value
            continue

        current = result[name]
        if isinstance(current, list):
            current.append(value)
        else:
            result[name] = [current, value]

    return result


def _first_model(root):
    for element in root.iter("model"):
        return element
    raise ConversionError("SDF does not contain a model element")


def _sensors_in_link_order(model):
    sensors = []
    for link in model.findall("link"):
        sensors.extend(link.findall("sensor"))
    return sensors


def sdf_to_json(sdf_text, host_type=None):
    """Return a JSON-compatible dictionary for one sensor in *sdf_text*."""
    try:
        root = ET.fromstring(sdf_text)
    except ET.ParseError as error:
        raise ConversionError(f"invalid SDF XML: {error}") from error

    if root.tag != "sdf":
        raise ConversionError("SDF root element must be sdf")

    sdf_version = root.get("version")
    if sdf_version is None:
        raise ConversionError("sdf.@version: missing required attribute")

    model = _first_model(root)
    sensors = _sensors_in_link_order(model)
    if not sensors:
        raise ConversionError("the first model does not contain a sensor in a link")

    if host_type is None:
        gazebo_type = sensors[0].get("type")
        if gazebo_type not in GAZEBO_TO_HOST:
            raise ConversionError(f"unknown Gazebo sensor type: {gazebo_type!r}")
        host_type = GAZEBO_TO_HOST[gazebo_type]
    elif host_type not in HOST_TO_GAZEBO:
        raise ConversionError(f"unknown host sensor type: {host_type!r}")

    requested_gazebo_type = HOST_TO_GAZEBO[host_type]
    sensor = None
    for candidate in sensors:
        if candidate.get("type") == requested_gazebo_type:
            sensor = candidate
            break

    if sensor is None:
        raise ConversionError(
            f"no sensor of Gazebo type {requested_gazebo_type!r} in the first model"
        )

    return {
        "type": host_type,
        "sdf_version": sdf_version,
        "sensor": _element_to_data(sensor),
    }


def _codepoint_in_ranges(codepoint, ranges):
    for start, end in ranges:
        if start <= codepoint <= end:
            return True
    return False


def _is_xml_name_start(character):
    if character in ":_":
        return True
    if "A" <= character <= "Z" or "a" <= character <= "z":
        return True

    ranges = (
        (0xC0, 0xD6),
        (0xD8, 0xF6),
        (0xF8, 0x2FF),
        (0x370, 0x37D),
        (0x37F, 0x1FFF),
        (0x200C, 0x200D),
        (0x2070, 0x218F),
        (0x2C00, 0x2FEF),
        (0x3001, 0xD7FF),
        (0xF900, 0xFDCF),
        (0xFDF0, 0xFFFD),
        (0x10000, 0xEFFFF),
    )
    return _codepoint_in_ranges(ord(character), ranges)


def _is_xml_name_character(character):
    if _is_xml_name_start(character):
        return True
    if character in "-." or "0" <= character <= "9":
        return True

    ranges = (
        (0xB7, 0xB7),
        (0x300, 0x36F),
        (0x203F, 0x2040),
    )
    return _codepoint_in_ranges(ord(character), ranges)


def _validate_xml_name(name, path):
    if not name:
        raise ConversionError(f"{path}: empty XML name")
    if not _is_xml_name_start(name[0]):
        raise ConversionError(f"{path}: invalid XML name {name!r}")
    for character in name[1:]:
        if not _is_xml_name_character(character):
            raise ConversionError(f"{path}: invalid XML name {name!r}")


def _format_scalar(value, path):
    if value is None:
        raise ConversionError(f"{path}: null is not representable in SDF")
    if isinstance(value, bool):
        raise ConversionError(f"{path}: JSON booleans are not representable in SDF")
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ConversionError(f"{path}: non-finite numbers are not representable in SDF")
        return repr(value)
    raise ConversionError(f"{path}: unsupported JSON value")


def _join_path(path, name):
    if path:
        return path + "." + name
    return name


def _append_child(parent, name, value, path):
    if isinstance(value, list):
        if not value:
            raise ConversionError(f"{path}: an empty array is not representable in SDF")
        for index, item in enumerate(value):
            item_path = f"{path}[{index}]"
            if isinstance(item, list):
                raise ConversionError(f"{item_path}: nested arrays are not allowed")
            parent.append(_data_to_element(name, item, item_path))
        return

    parent.append(_data_to_element(name, value, path))


def _data_to_element(name, value, path):
    _validate_xml_name(name, path)
    element = ET.Element(name)

    if not isinstance(value, dict):
        if isinstance(value, list):
            raise ConversionError(f"{path}: nested arrays are not allowed")
        element.text = _format_scalar(value, path)
        return element

    for key, item in value.items():
        key_path = _join_path(path, key)
        if key.startswith("@"):
            attribute_name = key[1:]
            _validate_xml_name(attribute_name, key_path)
            if isinstance(item, (dict, list)):
                raise ConversionError(f"{key_path}: an attribute must be a scalar")
            element.set(attribute_name, _format_scalar(item, key_path))
            continue

        if key == "#text":
            if isinstance(item, (dict, list)):
                raise ConversionError(f"{key_path}: element text must be a scalar")
            element.text = _format_scalar(item, key_path)
            continue

        _validate_xml_name(key, key_path)
        _append_child(element, key, item, key_path)

    return element


def _validate_top_level(document):
    if not isinstance(document, dict):
        raise ConversionError("$: top level must be a JSON object")

    if "sensor" not in document:
        raise ConversionError("sensor: missing required key")
    if "type" not in document:
        raise ConversionError("type: missing required key")
    if "sdf_version" not in document:
        raise ConversionError("sdf_version: missing required key")

    host_type = document["type"]
    if not isinstance(host_type, str) or host_type not in HOST_TO_GAZEBO:
        raise ConversionError(f"type: unknown host sensor type {host_type!r}")

    version = document["sdf_version"]
    if not isinstance(version, str):
        raise ConversionError("sdf_version: value must be a string")

    sensor = document["sensor"]
    if not isinstance(sensor, dict):
        if sensor is None:
            raise ConversionError("sensor: null is not representable in SDF")
        if isinstance(sensor, bool):
            raise ConversionError("sensor: JSON booleans are not representable in SDF")
        raise ConversionError("sensor: value must be a JSON object")


def json_to_sdf(document):
    """Return SDF XML text for a JSON-compatible sensor dictionary."""
    _validate_top_level(document)

    root = ET.Element("sdf", {"version": document["sdf_version"]})
    model = ET.SubElement(root, "model", {"name": "hakoniwa"})
    link = ET.SubElement(model, "link", {"name": "link"})
    sensor = _data_to_element("sensor", document["sensor"], "sensor")
    link.append(sensor)

    ET.indent(root, space="  ")
    xml_body = ET.tostring(root, encoding="unicode", short_empty_elements=True)
    return '<?xml version="1.0"?>\n' + xml_body + "\n"


def _read_text(path):
    try:
        return Path(path).read_text(encoding="utf-8")
    except OSError as error:
        raise ConversionError(f"cannot read {path}: {error}") from error


def _write_text(path, text):
    if path is None:
        sys.stdout.write(text)
        return

    try:
        Path(path).write_text(text, encoding="utf-8")
    except OSError as error:
        raise ConversionError(f"cannot write {path}: {error}") from error


def _run_to_json(args):
    sdf_text = _read_text(args.input)
    document = sdf_to_json(sdf_text, args.type)
    output = json.dumps(document, indent=2, ensure_ascii=False) + "\n"
    _write_text(args.output, output)


def _load_json(path):
    text = _read_text(path)
    try:
        return json.loads(text)
    except json.JSONDecodeError as error:
        raise ConversionError(f"invalid JSON in {path}: {error}") from error


def _run_to_sdf(args):
    document = _load_json(args.input)
    output = json_to_sdf(document)
    _write_text(args.output, output)


def _build_argument_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    to_json_parser = subparsers.add_parser("to-json", help="convert SDF to JSON")
    to_json_parser.add_argument("input", help="input SDF file")
    to_json_parser.add_argument("--type", help="Hakoniwa host sensor type")
    to_json_parser.add_argument("-o", "--output", help="output JSON file")
    to_json_parser.set_defaults(run=_run_to_json)

    to_sdf_parser = subparsers.add_parser("to-sdf", help="convert JSON to SDF")
    to_sdf_parser.add_argument("input", help="input JSON file")
    to_sdf_parser.add_argument("-o", "--output", help="output SDF file")
    to_sdf_parser.set_defaults(run=_run_to_sdf)

    return parser


def main(argv=None):
    parser = _build_argument_parser()
    args = parser.parse_args(argv)
    try:
        args.run(args)
    except ConversionError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
