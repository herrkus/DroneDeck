"""parammeta.py -- optional parameter metadata (units / range / description / enum).

QGC makes parameters usable by showing each one's units, valid range, description, and enum labels.
That metadata is NOT on the wire -- QGC bundles the autopilot's own definition files. DroneDeck loads
the SAME authoritative files the user can supply: ArduPilot's apm.pdef.xml or a PX4 parameters.json.
There are deliberately NO hardcoded guesses -- wrong range/units would be worse than none (it would
flag valid values as out-of-range). With no file loaded, get() returns None and the editor shows raw
values exactly as before.
"""
import json
import os
import xml.etree.ElementTree as ET


class ParamMeta:
    def __init__(self):
        self._meta = {}          # NAME -> {desc, units, min, max, values{int:str}, reboot}

    def __len__(self):
        return len(self._meta)

    def get(self, name):
        return self._meta.get(name)

    def load(self, path):
        """Auto-detect ArduPilot XML vs PX4 JSON by content, load it, and return how many params it added."""
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            head = f.read(4096).lstrip()
        before = len(self._meta)
        if head.startswith("<"):
            self.load_apm_xml(path)
        else:
            self.load_px4_json(path)
        return len(self._meta) - before

    def load_apm_xml(self, path):
        """Parse ArduPilot's apm.pdef.xml (any nesting). Fields: Units, Range 'min max', Values
        'code:label,...', RebootRequired."""
        root = ET.parse(path).getroot()
        for p in root.iter("param"):
            name = p.get("name") or ""
            if ":" in name:                          # e.g. "ArduCopter:ARMING_CHECK" -> "ARMING_CHECK"
                name = name.split(":")[-1]
            if not name:
                continue
            entry = {"desc": p.get("documentation") or p.get("humanName") or "",
                     "units": None, "min": None, "max": None, "values": None, "reboot": False}
            for fld in p.findall("field"):
                fname, txt = fld.get("name"), (fld.text or "").strip()
                if fname == "Units":
                    entry["units"] = txt or None
                elif fname == "Range":
                    parts = txt.replace(",", " ").split()
                    if len(parts) >= 2:
                        entry["min"], entry["max"] = _f(parts[0]), _f(parts[1])
                elif fname in ("Values", "Bitmask"):
                    entry["values"] = _parse_codes(txt)
                elif fname == "RebootRequired":
                    entry["reboot"] = txt.lower().startswith("t")
            # <values><value code="0">Disabled</value></values> form
            vals = p.find("values")
            if vals is not None and entry["values"] is None:
                got = {}
                for v in vals.findall("value"):
                    try:
                        got[int(v.get("code"))] = (v.text or "").strip()
                    except (TypeError, ValueError):
                        pass
                if got:
                    entry["values"] = got
            self._meta[name] = entry

    def load_px4_json(self, path):
        """Parse a PX4 parameters.json (the 'parameters' list; units/min/max/shortDesc/values)."""
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            data = json.load(f)
        for p in data.get("parameters", []):
            name = p.get("name")
            if not name:
                continue
            values = None
            if isinstance(p.get("values"), list):
                values = {}
                for v in p["values"]:
                    try:
                        values[int(v["value"])] = str(v.get("description", ""))
                    except (KeyError, ValueError, TypeError):
                        pass
                values = values or None
            self._meta[name] = {
                "desc": p.get("longDesc") or p.get("shortDesc") or "",
                "units": p.get("units") or None,
                "min": _f(p.get("min")), "max": _f(p.get("max")),
                "values": values, "reboot": bool(p.get("rebootRequired")),
            }

    def tooltip(self, name):
        """A human tooltip for a parameter, or '' if no metadata is loaded for it."""
        m = self._meta.get(name)
        if not m:
            return ""
        lines = [f"{name}"]
        if m["desc"]:
            lines.append(m["desc"])
        bits = []
        if m["units"]:
            bits.append(f"units: {m['units']}")
        if m["min"] is not None or m["max"] is not None:
            bits.append(f"range: {m['min']} .. {m['max']}")
        if m["reboot"]:
            bits.append("reboot required")
        if bits:
            lines.append("  |  ".join(bits))
        if m["values"]:
            lines.append("values: " + ", ".join(f"{k}={v}" for k, v in sorted(m["values"].items())))
        return "\n".join(lines)

    def out_of_range(self, name, value):
        """True if `value` is outside the documented [min, max] for `name` (False if unknown)."""
        m = self._meta.get(name)
        if not m:
            return False
        try:
            v = float(value)
        except (TypeError, ValueError):
            return False
        if m["min"] is not None and v < m["min"]:
            return True
        if m["max"] is not None and v > m["max"]:
            return True
        return False


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def _parse_codes(txt):
    """'0:Disabled,1:Enabled' (or newline/space separated) -> {0:'Disabled', 1:'Enabled'}."""
    out = {}
    for pair in txt.replace("\n", ",").split(","):
        pair = pair.strip()
        if ":" not in pair:
            continue
        code, _, label = pair.partition(":")
        try:
            out[int(code.strip())] = label.strip()
        except ValueError:
            pass
    return out or None


def default_path():
    """A conventional place to drop a metadata file so it auto-loads (apm.pdef.xml / px4 params.json)."""
    for p in (os.path.expanduser("~/.config/dronedeck/apm.pdef.xml"),
              os.path.expanduser("~/.config/dronedeck/parameters.json")):
        if os.path.isfile(p):
            return p
    return None
