import json
from pathlib import Path
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET


TOOLS_DIR = Path(__file__).resolve().parent
REPOSITORY_DIR = TOOLS_DIR.parent
CONFIG_DIR = REPOSITORY_DIR / "config"
sys.path.insert(0, str(TOOLS_DIR))

import sdf_json


HOST_TYPES = {
    "baro": "baro",
    "ft": "force_torque",
    "gps": "gps",
    "imu": "imu",
    "mag": "mag",
}


def sensor_from_xml(xml_text):
    root = ET.fromstring(xml_text)
    model = next(root.iter("model"))
    for link in model.findall("link"):
        sensor = link.find("sensor")
        if sensor is not None:
            return sensor
    raise AssertionError("test XML has no sensor")


def text_values_equal(left, right):
    left = (left or "").strip()
    right = (right or "").strip()
    try:
        return float(left) == float(right)
    except ValueError:
        return left == right


class SdfJsonTest(unittest.TestCase):
    maxDiff = None

    def assert_elements_equal(self, expected, actual):
        self.assertEqual(expected.tag, actual.tag)
        self.assertEqual(expected.attrib, actual.attrib)
        self.assertTrue(
            text_values_equal(expected.text, actual.text),
            msg=f"different text in <{expected.tag}>",
        )

        expected_children = list(expected)
        actual_children = list(actual)
        self.assertEqual(
            [child.tag for child in expected_children],
            [child.tag for child in actual_children],
        )
        for expected_child, actual_child in zip(expected_children, actual_children):
            self.assert_elements_equal(expected_child, actual_child)

    def base_document(self):
        return {
            "type": "imu",
            "sdf_version": "1.11",
            "sensor": {
                "@name": "imu",
                "@type": "imu",
                "update_rate": 250,
            },
        }

    def assert_rejected_at(self, document, path):
        with self.assertRaises(sdf_json.ConversionError) as context:
            sdf_json.json_to_sdf(document)
        self.assertIn(path, str(context.exception))

    def test_sdf_json_sdf_round_trip_for_every_fixture(self):
        for sdf_path in sorted(CONFIG_DIR.glob("*.sdf")):
            with self.subTest(sdf=sdf_path.name):
                original_text = sdf_path.read_text(encoding="utf-8")
                host_prefix = sdf_path.stem.split("-", 1)[0]
                document = sdf_json.sdf_to_json(
                    original_text,
                    HOST_TYPES[host_prefix],
                )
                result_text = sdf_json.json_to_sdf(document)
                self.assert_elements_equal(
                    sensor_from_xml(original_text),
                    sensor_from_xml(result_text),
                )

    def test_json_sdf_json_round_trip_for_generated_examples(self):
        for json_path in sorted(CONFIG_DIR.glob("*.json")):
            with self.subTest(json=json_path.name):
                original = json.loads(json_path.read_text(encoding="utf-8"))
                sdf_text = sdf_json.json_to_sdf(original)
                result = sdf_json.sdf_to_json(sdf_text, original["type"])
                self.assertEqual(original, result)

    def test_imu_noise_expected_values(self):
        sdf_path = CONFIG_DIR / "imu-noise.sdf"
        document = sdf_json.sdf_to_json(
            sdf_path.read_text(encoding="utf-8"),
            "imu",
        )
        sensor = document["sensor"]
        noise = sensor["imu"]["angular_velocity"]["x"]["noise"]
        self.assertEqual("gaussian", noise["@type"])
        self.assertEqual(0.0008726646, noise["stddev"])
        self.assertIsInstance(noise["stddev"], float)
        self.assertEqual(250, sensor["update_rate"])
        self.assertIsInstance(sensor["update_rate"], int)
        self.assertEqual("/hakoniwa/imu", sensor["topic"])

    def test_repeated_elements_become_an_array(self):
        sdf_text = """<?xml version="1.0"?>
<sdf version="1.11">
  <model name="m">
    <link name="link">
      <sensor name="imu" type="imu">
        <item>first</item>
        <item>second</item>
      </sensor>
    </link>
  </model>
</sdf>
"""
        document = sdf_json.sdf_to_json(sdf_text)
        self.assertEqual(["first", "second"], document["sensor"]["item"])

    def test_invalid_xml_name_is_rejected_with_path(self):
        document = self.base_document()
        document["sensor"]["imu"] = {"1invalid": "value"}
        self.assert_rejected_at(document, "sensor.imu.1invalid")

    def test_nested_array_is_rejected_with_path(self):
        document = self.base_document()
        document["sensor"]["sample"] = [[1, 2]]
        self.assert_rejected_at(document, "sensor.sample[0]")

    def test_null_is_rejected_with_path(self):
        document = self.base_document()
        document["sensor"]["imu"] = {"value": None}
        self.assert_rejected_at(document, "sensor.imu.value")

    def test_boolean_is_rejected_with_path(self):
        document = self.base_document()
        document["sensor"]["enabled"] = True
        self.assert_rejected_at(document, "sensor.enabled")

    def test_missing_sensor_is_rejected_with_path(self):
        document = self.base_document()
        del document["sensor"]
        self.assert_rejected_at(document, "sensor")

    def test_unknown_host_type_is_rejected_with_path(self):
        document = self.base_document()
        document["type"] = "camera"
        self.assert_rejected_at(document, "type")

    def test_type_is_inferred_from_first_sensor(self):
        sdf_text = """<sdf version="1.10">
  <model name="m">
    <link name="link">
      <sensor name="mag" type="magnetometer" />
    </link>
  </model>
</sdf>"""
        document = sdf_json.sdf_to_json(sdf_text)
        self.assertEqual("mag", document["type"])
        self.assertEqual("magnetometer", document["sensor"]["@type"])

    def test_unknown_inferred_type_is_an_error(self):
        sdf_text = """<sdf version="1.10">
  <model name="m">
    <link name="link">
      <sensor name="camera" type="camera" />
    </link>
  </model>
</sdf>"""
        with self.assertRaises(sdf_json.ConversionError) as context:
            sdf_json.sdf_to_json(sdf_text)
        self.assertIn("camera", str(context.exception))

    def test_unknown_cli_type_exits_one_and_writes_stderr(self):
        command = [
            sys.executable,
            str(TOOLS_DIR / "sdf_json.py"),
            "to-json",
            str(CONFIG_DIR / "imu-ideal.sdf"),
            "--type",
            "camera",
        ]
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(1, result.returncode)
        self.assertEqual("", result.stdout)
        self.assertIn("camera", result.stderr)

    def test_requested_type_selects_first_matching_sensor(self):
        sdf_text = """<sdf version="1.11">
  <model name="m">
    <link name="first">
      <sensor name="mag" type="magnetometer" />
      <sensor name="imu-one" type="imu" />
    </link>
    <link name="second">
      <sensor name="imu-two" type="imu" />
    </link>
  </model>
</sdf>"""
        document = sdf_json.sdf_to_json(sdf_text, "imu")
        self.assertEqual("imu-one", document["sensor"]["@name"])

    def test_attributes_and_text_are_preserved(self):
        sdf_text = """<sdf version="1.11">
  <model name="m">
    <link name="link">
      <sensor name="imu" type="imu">
        <value unit="m">1.25</value>
      </sensor>
    </link>
  </model>
</sdf>"""
        document = sdf_json.sdf_to_json(sdf_text)
        self.assertEqual(
            {"@unit": "m", "#text": 1.25},
            document["sensor"]["value"],
        )
        result = sdf_json.sdf_to_json(sdf_json.json_to_sdf(document))
        self.assertEqual(document, result)


if __name__ == "__main__":
    unittest.main()
