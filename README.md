# uge2xm

Convert [hUGETracker](https://github.com/SuperDisk/hUGETracker) `.uge` songs to FastTracker II `.xm` (4 channels, Game Boy-style embedded samples).

## Usage

```bash
# Basic usage:
python3 uge2xm.py input.uge [output.xm]

# Recommended settings:
python3 uge2xm.py --c-envelope-volslide --bake-noise-envelope=normalized --remove-unused input.uge [output.xm]
```

If `output.xm` is omitted, it writes next to the input with a `.xm` extension.

For a complete list of options, run:

```bash
python3 uge2xm.py --help
```

Functionality is still limited; patches and pull requests are welcome.

## License

[Zlib](LICENSE). Each `.py` file starts with a short copyright + SPDX line; full text is in `LICENSE`. To opt out for one file, add `# uge2xm-license-header: skip` near the top (see `scripts/add_license_headers.py` docstring).

To (re)apply headers after adding files:

```bash
python3 scripts/add_license_headers.py
```

With [pre-commit](https://pre-commit.com): `pip install pre-commit && pre-commit install` in this repo, then staged Python files get headers on commit (see `.pre-commit-config.yaml`).
