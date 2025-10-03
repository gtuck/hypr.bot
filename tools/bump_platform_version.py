#!/usr/bin/env python3
import sys, re, json, pathlib, subprocess, os

try:
    import yaml  # type: ignore
except Exception:
    print("PyYAML is required. pip install pyyaml", file=sys.stderr)
    sys.exit(1)

CONFIGS = ["config.yaml", "config.json"]
VERSION_KEY_PATH = ["identity", "platform_version"]

def bump_semver(v):
    m = re.match(r"^v(\d+)\.(\d+)\.(\d+)$", v)
    if not m: raise ValueError(f"Bad platform_version: {v}")
    major, minor, patch = map(int, m.groups())
    return f"v{major}.{minor}.{patch+1}"

def update_yaml(p):
    data = yaml.safe_load(p.read_text())
    d = data
    for k in VERSION_KEY_PATH[:-1]:
        d = d[k]
    d[VERSION_KEY_PATH[-1]] = bump_semver(d[VERSION_KEY_PATH[-1]])
    p.write_text(yaml.safe_dump(data, sort_keys=False))
    return data["identity"]["platform_version"]

def update_json(p):
    data = json.loads(p.read_text())
    d = data
    for k in VERSION_KEY_PATH[:-1]:
        d = d[k]
    d[VERSION_KEY_PATH[-1]] = bump_semver(d[VERSION_KEY_PATH[-1]])
    p.write_text(json.dumps(data, indent=2))
    return data["identity"]["platform_version"]

def main():
    repo = pathlib.Path(".")
    # Bump both if present
    touched = [c for c in CONFIGS if (repo / c).exists()]
    if not touched:
        print("No config files found; skipping bump.")
        return
    new_versions = []
    for cfg in touched:
        p = repo / cfg
        if cfg.endswith(".yaml"): new_versions.append(update_yaml(p))
        else: new_versions.append(update_json(p))
    # Output for GitHub Actions
    msg = f"chore: bump platform_version to {new_versions[-1]}"
    github_output = pathlib.Path(os.environ.get("GITHUB_OUTPUT", "/dev/stdout"))
    with github_output.open("a") as f:
        f.write(f"msg={msg}\n")
        f.write(f"version={new_versions[-1]}\n")

if __name__ == "__main__":
    main()
