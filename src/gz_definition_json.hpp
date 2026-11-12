#pragma once

// JSON sensor definitions (not part of the public interface).
//
// A host that reads definition files as JSON (the Hakoniwa Robot Runtime) gets
// them from an SDFormat <sensor> with tools/sdf_json.py. This file turns such a
// JSON definition back into SDFormat text for gz-sensors. The mapping is one to
// one (README "JSON definition files"):
//   {"type": host type, "sdf_version": "1.11", "sensor": <sensor element>}
//   "@name" -> attribute, "#text" -> text, other key -> child element,
//   array -> repeated child elements, number or string -> leaf text, "" -> empty element.

#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

namespace hakoniwa::sensor_adapter::gazebo::detail {

using Json = nlohmann::ordered_json;

// The host's sensor type name of a gz-sensors type ("air_pressure" -> "baro").
inline std::string host_type_of(const std::string& gz_type)
{
    if (gz_type == "magnetometer") {
        return "mag";
    }
    if (gz_type == "air_pressure") {
        return "baro";
    }
    if (gz_type == "navsat") {
        return "gps";
    }
    return gz_type;  // imu, force_torque
}

inline bool is_json_definition(const std::string& path)
{
    const std::string suffix = ".json";
    return path.size() >= suffix.size() && path.compare(path.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// XML names: a letter or '_' first, then letters, digits, '_', '-' or '.'.
inline bool is_xml_name(const std::string& name)
{
    if (name.empty()) {
        return false;
    }
    const unsigned char first = static_cast<unsigned char>(name[0]);
    if (!std::isalpha(first) && first != '_') {
        return false;
    }
    for (const char c : name) {
        const unsigned char u = static_cast<unsigned char>(c);
        if (!std::isalnum(u) && c != '_' && c != '-' && c != '.') {
            return false;
        }
    }
    return true;
}

inline std::string xml_escape(const std::string& text)
{
    std::string out;
    for (const char c : text) {
        switch (c) {
        case '&':
            out += "&amp;";
            break;
        case '<':
            out += "&lt;";
            break;
        case '>':
            out += "&gt;";
            break;
        case '"':
            out += "&quot;";
            break;
        default:
            out += c;
        }
    }
    return out;
}

// Leaf text of a number or a string; false (with error) for anything else.
inline bool leaf_text(const Json& value, const std::string& path, std::string& text, std::string& error)
{
    if (value.is_string()) {
        text = value.get<std::string>();
        return true;
    }
    if (value.is_number()) {
        text = value.dump();  // integers as is, floats in the shortest round-trip form
        return true;
    }
    if (value.is_boolean()) {
        error = path + ": a boolean is not SDFormat text (write \"true\" or \"false\")";
    } else if (value.is_null()) {
        error = path + ": null is not allowed";
    } else {
        error = path + ": an array inside an array is not allowed";
    }
    return false;
}

inline bool write_element(const std::string& name, const Json& value, const std::string& path, std::string& xml,
                          std::string& error);

// Writes <name ...>...</name> for an object value.
inline bool write_object(const std::string& name, const Json& object, const std::string& path, std::string& xml,
                         std::string& error)
{
    std::string attributes;
    std::string text;
    std::string children;
    for (const auto& [key, value] : object.items()) {
        const std::string child_path = path + "." + key;
        if (key == "#text") {
            if (!leaf_text(value, child_path, text, error)) {
                return false;
            }
            continue;
        }
        if (!key.empty() && key[0] == '@') {
            const std::string attribute = key.substr(1);
            if (!is_xml_name(attribute)) {
                error = child_path + ": not an XML attribute name";
                return false;
            }
            if (!value.is_string()) {
                error = child_path + ": an attribute value must be a string";
                return false;
            }
            attributes += " " + attribute + "=\"" + xml_escape(value.get<std::string>()) + "\"";
            continue;
        }
        if (value.is_array()) {
            for (std::size_t i = 0; i < value.size(); ++i) {
                const std::string item_path = child_path + "[" + std::to_string(i) + "]";
                if (value[i].is_array()) {
                    error = item_path + ": an array inside an array is not allowed";
                    return false;
                }
                if (!write_element(key, value[i], item_path, children, error)) {
                    return false;
                }
            }
            continue;
        }
        if (!write_element(key, value, child_path, children, error)) {
            return false;
        }
    }
    xml += "<" + name + attributes + ">" + xml_escape(text) + children + "</" + name + ">";
    return true;
}

inline bool write_element(const std::string& name, const Json& value, const std::string& path, std::string& xml,
                          std::string& error)
{
    if (!is_xml_name(name)) {
        error = path + ": not an XML element name";
        return false;
    }
    if (value.is_object()) {
        return write_object(name, value, path, xml, error);
    }
    std::string text;
    if (!leaf_text(value, path, text, error)) {
        return false;
    }
    xml += "<" + name + ">" + xml_escape(text) + "</" + name + ">";
    return true;
}

// Reads a JSON definition and writes the SDFormat text of one model with that sensor.
// gz_type is the gz-sensors type the adapter needs ("imu", "air_pressure", ...).
inline bool sdf_from_json_definition(const std::string& path, const std::string& gz_type, std::string& sdf_text,
                                     std::string& error)
{
    std::ifstream file(path);
    if (!file) {
        error = "cannot open " + path;
        return false;
    }
    Json root;
    try {
        root = Json::parse(file);
    } catch (const std::exception& e) {
        error = path + ": " + e.what();
        return false;
    }
    if (!root.is_object() || !root.contains("sensor") || !root.at("sensor").is_object()) {
        error = path + ": the definition needs a \"sensor\" object";
        return false;
    }
    const std::string host_type = host_type_of(gz_type);
    if (root.contains("type") && root.at("type") != host_type) {
        error = path + ": type must be \"" + host_type + "\"";
        return false;
    }
    const Json& sensor = root.at("sensor");
    if (sensor.contains("@type") && sensor.at("@type") != gz_type) {
        error = path + ": sensor.@type must be \"" + gz_type + "\"";
        return false;
    }
    Json element = sensor;
    if (!element.contains("@name")) {
        element["@name"] = host_type;
    }
    element["@type"] = gz_type;
    std::string version = "1.11";
    if (root.contains("sdf_version")) {
        if (!root.at("sdf_version").is_string()) {
            error = path + ": sdf_version must be a string";
            return false;
        }
        version = root.at("sdf_version").get<std::string>();
    }
    std::string body;
    if (!write_element("sensor", element, "sensor", body, error)) {
        error = path + ": " + error;
        return false;
    }
    sdf_text = "<?xml version=\"1.0\"?><sdf version=\"" + xml_escape(version) +
               "\"><model name=\"hakoniwa\"><link name=\"link\">" + body + "</link></model></sdf>";
    return true;
}

}  // namespace hakoniwa::sensor_adapter::gazebo::detail
