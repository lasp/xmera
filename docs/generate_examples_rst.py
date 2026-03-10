"""Generate RST documentation for Python example scripts.

Extracts module docstrings using ast.get_docstring() and embeds them directly
in the generated RST files. Optionally runs scenarios to generate plot images.
"""

import ast
import os
import argparse
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)


def extract_docstring(py_path):
    """Extract the module-level docstring from a Python file using AST."""
    try:
        with open(py_path, 'r', encoding='utf-8', errors='ignore') as f:
            source = f.read()
        tree = ast.parse(source)
        return ast.get_docstring(tree) or ""
    except (SyntaxError, ValueError) as e:
        logger.warning(f"  [warn] Could not parse {py_path}: {e}")
        return ""


def write_module_rst(module_name, py_source_path, output_path, label=None):
    """Write an RST file for a single Python module, embedding its docstring."""
    if label is None:
        label = module_name

    docstring = extract_docstring(py_source_path)

    with open(output_path, 'w') as f:
        f.write(f".. _{label}:\n\n")
        f.write(f"{module_name}\n")
        f.write(f"{'=' * len(module_name)}\n\n")

        if docstring:
            f.write(docstring)
            f.write("\n\n")

    logger.info(f"  [file] Wrote module RST: {output_path}"
                + (" (with docstring)" if docstring else " (no docstring)"))


def write_index_rst(dir_name, module_rst_files, subdirs, output_path, is_root=False, rel_path=None):
    if is_root:
        label = "examples"
    elif rel_path:
        label = f"Folder_{rel_path.replace(os.sep, '_')}"
    else:
        label = f"Folder_{dir_name}"
    with open(output_path, 'w') as f:
        f.write(f""".. _{label}:

{dir_name}
{'=' * len(dir_name)}


""")
        # Deduplicate rst files (by stem) in case of duplicates
        seen = set()
        unique_rst_files = []
        for rst in module_rst_files:
            if rst not in seen:
                seen.add(rst)
                unique_rst_files.append(rst)

        if unique_rst_files:
            f.write(""".. toctree::
   :maxdepth: 1
   :caption: Files:

""")
            for rst in unique_rst_files:
                f.write(f"   {rst}\n")

        if subdirs:
            f.write(""".. toctree::
   :maxdepth: 1
   :caption: Directories:

""")
            for subdir in subdirs:
                f.write(f"   {subdir}/index\n")

    logger.info(f"  [index] Wrote index RST: {output_path}")


def generate_rst_tree(source_root, output_root):
    """
    Walks through `source_root`, and generates .rst files under `output_root`, preserving the directory structure.
    """
    logger.info(f"Starting RST generation from '{source_root}' to '{output_root}'\n")

    for current_dir, subdirs, files in os.walk(source_root):
        rel_path = os.path.relpath(current_dir, source_root)
        is_root = rel_path == "."

        output_dir = os.path.join(output_root, "" if is_root else rel_path)
        os.makedirs(output_dir, exist_ok=True)

        logger.info(f"[dir] Processing directory: {current_dir}")
        logger.info(f"  [output] -> {output_dir}")

        py_files = [f for f in files if f.endswith('.py') and not f.startswith('__')]
        module_rst_files = []

        for py_file in py_files:
            module_name = os.path.splitext(py_file)[0]
            rst_filename = f"{module_name}.rst"
            module_rst_files.append(rst_filename)

            # Use path-based label to avoid duplicate labels across directories
            if is_root:
                label = module_name
            else:
                label = f"{rel_path.replace(os.sep, '_')}_{module_name}"

            py_source_path = os.path.join(current_dir, py_file)
            output_rst_path = os.path.join(output_dir, rst_filename)
            write_module_rst(module_name, py_source_path, output_rst_path, label=label)

        exclude_dirs = {'__pycache__', '_VizFiles', 'Support', 'dataForExamples'}
        clean_subdirs = [d for d in subdirs if not d.startswith('__') and not d.startswith('.') and d not in exclude_dirs]
        subdirs[:] = clean_subdirs

        dir_display_name = os.path.basename(current_dir) if not is_root else "Examples"
        index_rst_path = os.path.join(output_dir, "index.rst")
        write_index_rst(dir_display_name, module_rst_files, clean_subdirs, index_rst_path, is_root=is_root, rel_path=rel_path)

    logger.info("\nRST generation complete.")


def generate_plots(source_root, image_output_dir):
    """Run scenario scripts and save their matplotlib figures as images.

    Only runs scenarios that define a `run()` function returning a figureList dict.
    Saves each figure as a PNG in image_output_dir.
    """
    import importlib.util
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt

    os.makedirs(image_output_dir, exist_ok=True)

    for py_file in sorted(os.listdir(source_root)):
        if not py_file.endswith('.py') or py_file.startswith('__'):
            continue

        module_name = os.path.splitext(py_file)[0]
        py_path = os.path.join(source_root, py_file)

        # Check if the module has a run() function
        try:
            with open(py_path, 'r', encoding='utf-8', errors='ignore') as f:
                source = f.read()
            tree = ast.parse(source)
        except (SyntaxError, ValueError):
            continue

        has_run = any(
            isinstance(node, ast.FunctionDef) and node.name == 'run'
            for node in ast.walk(tree)
        )
        if not has_run:
            continue

        logger.info(f"  [plot] Running {module_name}...")
        try:
            spec = importlib.util.spec_from_file_location(module_name, py_path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)

            # Call run() with show_plots=False
            import inspect
            sig = inspect.signature(mod.run)
            kwargs = {}
            if 'show_plots' in sig.parameters:
                kwargs['show_plots'] = False

            result = mod.run(**kwargs)

            # Extract figureList from result
            figure_list = None
            if isinstance(result, dict):
                figure_list = result
            elif isinstance(result, tuple):
                for item in result:
                    if isinstance(item, dict):
                        figure_list = item
                        break

            if figure_list:
                for fig_name, fig in figure_list.items():
                    fig_path = os.path.join(image_output_dir, f"{fig_name}.png")
                    fig.savefig(fig_path, dpi=150, bbox_inches='tight')
                    logger.info(f"    Saved {fig_path}")

            plt.close('all')
        except Exception as e:
            logger.warning(f"  [warn] Failed to run {module_name}: {e}")
            plt.close('all')


def main():
    parser = argparse.ArgumentParser(description="Generate .rst documentation tree for Python project.")
    parser.add_argument("source", help="Source directory containing Python files")
    parser.add_argument("destination", help="Destination directory for generated .rst files")
    parser.add_argument("--generate-plots", action="store_true",
                        help="Run scenarios and generate plot images")
    parser.add_argument("--plot-output", default=None,
                        help="Directory to save generated plot images")

    args = parser.parse_args()
    generate_rst_tree(args.source, args.destination)

    if args.generate_plots:
        plot_dir = args.plot_output or os.path.join(args.destination, '..', '_images', 'scenarios')
        generate_plots(args.source, plot_dir)


if __name__ == "__main__":
    main()
