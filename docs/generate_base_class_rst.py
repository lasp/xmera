"""Generate RST files for base classes found in _GeneralModuleFiles directories.

Scans _GeneralModuleFiles dirs under src/ for .rst files, combines them with
Doxygen class documentation, and outputs to docs/source/api/baseClasses/.
Also handles moduleTemplates.
"""

import os
import re
from pathlib import Path


# Map of RST filename (without extension) to primary C++ class name for doxygenclass
# This is used when the class name differs from the filename
CLASS_NAME_MAP = {
    "dynamicEffector": "DynamicEffector",
    "dynamicObject": "DynamicObject",
    "dynParamManager": "StateVector",
    "gravityEffector": "GravityEffector",
    "hubEffector": "HubEffector",
    "stateData": "StateData",
    "stateEffector": "StateEffector",
    "stateVecIntegrator": "StateVecIntegrator",
    "svIntegratorRK4": "svIntegratorRK4",
    "atmosphereBase": "AtmosphereBase",
    "magneticFieldBase": "MagneticFieldBase",
    "dataNodeBase": "DataNodeBase",
    "dataStorageUnitBase": "DataStorageUnitBase",
    "powerNodeBase": "PowerNodeBase",
    "powerStorageBase": "PowerStorageBase",
    "dynamicModels": "DynamicsModel",
    "ekfInterface": "EkfInterface",
    "kalmanFilter": "KalmanFilter",
    "measurementModels": "MeasurementModel",
    "srukfInterface": "SRukfInterface",
    "stateModels": "State",
    "sys_model": "SysModel",
}


def generate_base_class_rst(rst_source_path, class_name, output_dir):
    """Generate a single RST for a base class, including source RST content."""
    basename = Path(rst_source_path).stem
    label = basename.lower()
    title = basename
    underline = "=" * len(title)

    # Read the source RST content (description from the module)
    source_content = ""
    try:
        source_content = Path(rst_source_path).read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        pass

    content = f".. _{label}:\n\n{title}\n{underline}\n\n"
    if source_content:
        content += source_content + "\n\n"
    content += f".. doxygenclass:: {class_name}\n   :members:\n"

    output_path = os.path.join(output_dir, f"{basename}.rst")
    with open(output_path, "w") as f:
        f.write(content)
    return f"{basename}.rst"


def generate_module_template_rst(rst_source_path, output_dir):
    """Generate RST for a module template."""
    basename = Path(rst_source_path).stem
    label = basename.lower()
    title = basename
    underline = "=" * len(title)

    source_content = ""
    try:
        source_content = Path(rst_source_path).read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        pass

    content = f".. _{label}:\n\n{title}\n{underline}\n\n"
    if source_content:
        content += source_content + "\n"

    output_path = os.path.join(output_dir, f"{basename}.rst")
    with open(output_path, "w") as f:
        f.write(content)
    return f"{basename}.rst"


def generate_index(rst_filenames, output_dir):
    """Generate index.rst for base class docs."""
    entries = sorted(os.path.splitext(f)[0] for f in rst_filenames)
    lines = [
        ".. _baseclasses:",
        "",
        "Base Classes",
        "============",
        "",
        ".. toctree::",
        "   :maxdepth: 1",
        "   :hidden:",
        "",
    ]
    lines.extend(f"   {e}" for e in entries)
    lines.append("")
    # Visible list of links (toctree is hidden for nav-tree only)
    lines.append("")
    for e in entries:
        lines.append(f"- :doc:`{e}`")
    lines.append("")

    with open(os.path.join(output_dir, "index.rst"), "w") as f:
        f.write("\n".join(lines))


def main():
    script_dir = Path(os.path.dirname(os.path.abspath(__file__)))
    src_root = script_dir / ".." / "src"
    output_dir = script_dir / "source" / "api" / "baseClasses"
    os.makedirs(output_dir, exist_ok=True)

    rst_filenames = []

    # Scan all _GeneralModuleFiles directories
    for gmf_dir in sorted(src_root.rglob("_GeneralModuleFiles")):
        if not gmf_dir.is_dir():
            continue
        for rst_file in sorted(gmf_dir.glob("*.rst")):
            basename = rst_file.stem
            class_name = CLASS_NAME_MAP.get(basename)
            if class_name:
                rst_fn = generate_base_class_rst(str(rst_file), class_name, str(output_dir))
                rst_filenames.append(rst_fn)
                print(f"  Generated {rst_fn} for {class_name}")
            else:
                # Generate without doxygenclass directive
                rst_fn = generate_module_template_rst(str(rst_file), str(output_dir))
                rst_filenames.append(rst_fn)
                print(f"  Generated {rst_fn} (no class mapping)")

    # Scan additional utility directories with RST files
    extra_dirs = [
        src_root / "architecture" / "utilities",
    ]
    for extra_dir in extra_dirs:
        if not extra_dir.is_dir():
            continue
        for rst_file in sorted(extra_dir.glob("*.rst")):
            basename = rst_file.stem
            class_name = CLASS_NAME_MAP.get(basename)
            if class_name:
                rst_fn = generate_base_class_rst(str(rst_file), class_name, str(output_dir))
            else:
                rst_fn = generate_module_template_rst(str(rst_file), str(output_dir))
            rst_filenames.append(rst_fn)
            print(f"  Generated {rst_fn} from {extra_dir}")

    # Handle moduleTemplates
    templates_dir = src_root / "moduleTemplates"
    if templates_dir.is_dir():
        # The _doc.rst for the templates folder — generate as moduleTemplates.rst
        doc_rst = templates_dir / "_doc.rst"
        if doc_rst.exists():
            source_content = doc_rst.read_text(encoding="utf-8").strip()
            title = "moduleTemplates"
            content = f".. _{title.lower()}:\n\n{title}\n{'=' * len(title)}\n\n{source_content}\n"
            out_path = output_dir / "moduleTemplates.rst"
            out_path.write_text(content)
            rst_filenames.append("moduleTemplates.rst")
            print(f"  Generated moduleTemplates.rst from _doc.rst")

        # Individual template modules
        for tmpl_dir in sorted(templates_dir.iterdir()):
            if tmpl_dir.is_dir() and not tmpl_dir.name.startswith("_"):
                for rst_file in sorted(tmpl_dir.glob("*.rst")):
                    class_name = CLASS_NAME_MAP.get(rst_file.stem)
                    if class_name:
                        rst_fn = generate_base_class_rst(str(rst_file), class_name, str(output_dir))
                    else:
                        rst_fn = generate_module_template_rst(str(rst_file), str(output_dir))
                    rst_filenames.append(rst_fn)
                    print(f"  Generated {rst_fn} from moduleTemplates")

    generate_index(rst_filenames, str(output_dir))
    print(f"Generated {len(rst_filenames)} base class RSTs + index in {output_dir}")


if __name__ == "__main__":
    main()
