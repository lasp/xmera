import os
import argparse
import re
from dataclasses import dataclass
from typing import List


@dataclass
class FileGroup:
    """Class for keeping track of documentation files."""
    rst_file_path: str
    underscore_filename: str
    class_rst_file_path: str
    categories: List[str]

    def class_rst_file_name(self) -> str:
        return "class_" + self.underscore_filename + ".rst"


def camel_case_to_underscore_string(filename):
    # This regex finds uppercase letters
    components = re.findall('[a-zA-Z][^A-Z]*', filename)
    return "_".join([component.lower() for component in components])


def get_list_from_location(data_list, match_string):
    """
    Returns a new list starting from the first occurrence of the match string.

    Args:
        data_list: The original list.
        match_string: The string to search for.

    Returns:
        A new list starting from the match, or an empty list if no match is found.
    """
    try:
        index = data_list.index(match_string)
        return data_list[index:]
    except ValueError:
        return []


def collect_rst_files(directory, exclude_patterns=None):
    """
    Recursively collect .rst files in the directory, excluding those matching the exclude_patterns regex.

    :param directory: The directory to start the search.
    :param exclude_patterns: A list of regex patterns. If any pattern matches a path, it will be excluded.
    :return: A list of tuples (file_path, dashed_filename)
    """
    if exclude_patterns is None:
        exclude_patterns = []

    # List to hold the full paths of all .rst files
    rst_files = []

    # Walk through the directory structure
    for root, _, files in os.walk(directory):
        # Check if the current directory matches any exclude pattern
        if any(re.search(pattern, root) for pattern in exclude_patterns):
            continue  # Skip this directory if it matches any exclude pattern

        for file in files:
            if file.endswith('.rst'):
                # Parse the filename into underscore format
                underscore_filename = camel_case_to_underscore_string(os.path.splitext(file)[0])  # Remove .rst before processing
                # Save both the file path and the modified filename
                rst_files.append(FileGroup(os.path.join(root, file),
                                           underscore_filename,
                                           "",
                                           os.path.relpath(root, os.path.split(directory)[0]))
                                 )

    return rst_files


def fill_matching_pairs(directory, rst_files):
    """
    For each .rst file found, search for a corresponding file with 'class' appended to its dashed filename
    and return a list of matching files pairs.

    :param directory: The base directory to search for the files.
    :param rst_files: List of tuples containing file paths and dashed filenames.
    :return:
    """

    # Walk through the directory and search for class files
    for root, _, files in os.walk(directory):
        print(files)
        break

    for file_group in rst_files:
        # Search for the class file in the directory structure
        for root, _, files in os.walk(directory):
            print(file_group.class_rst_file_name())
            if file_group.class_rst_file_name() in files:
                file_group.class_rst_file_path = os.path.join(root, file_group.class_rst_file_name())
                print(file_group.class_rst_file_path)
                break  # Stop after finding the first matching class file


def _namespace_c_variant_labels(content, title):
    """For _C variant modules, namespace internal labels to avoid duplicates.

    Transforms:
      :label: eq:X        -> :label: eq_c:X
      :eq:`eq:X`          -> :eq:`eq_c:X`
      .. _ModuleIO_X:     -> .. _ModuleIO_c_X:
      :ref:`ModuleIO_X`   -> :ref:`ModuleIO_c_X`
    """
    # Namespace equation labels
    content = re.sub(r':label:\s*eq:', ':label: eq_c:', content)
    content = re.sub(r':eq:`eq:', ':eq:`eq_c:', content)

    # Namespace section labels like .. _ModuleIO_X:
    content = re.sub(r'\.\. _ModuleIO_', '.. _ModuleIO_c_', content)
    content = re.sub(r':ref:`ModuleIO_', ':ref:`ModuleIO_c_', content)

    return content


def concatenate_rst_files(file1, file2, output_directory):
    """
    Concatenate the contents of two .rst files and save them to a new file in the output directory.

    :param file1: The first .rst file.
    :param file2: The second .rst file.
    :param output_directory: Path to the directory where the new combined file should be saved.
    """
    # Read the contents of both files
    content1 = ""
    try:
        with open(file1, 'r', encoding='utf-8') as f1:
            content1 = f1.read()
    except FileNotFoundError:
        print("File 1 not found, continuing...")

    content2 = ""
    try:
        with open(file2, 'r', encoding='utf-8') as f2:
            content2 = f2.read()
    except FileNotFoundError:
        print("File 2 not found, continuing...")

    title = os.path.splitext(os.path.basename(file1))[0]

    # For _doc files, use the parent directory name to avoid generating
    # ".. __doc:" which is an invalid RST anonymous hyperlink target
    if title == "_doc":
        parent_name = os.path.basename(os.path.dirname(file1))
        title = parent_name
        label = ".. _folder_" + parent_name.lower() + ":"
    else:
        label = ".. _" + title.lower() + ":"

    # For _C variant modules, namespace internal labels to avoid duplicates
    is_c_variant = title.endswith('_C')
    if is_c_variant:
        content1 = _namespace_c_variant_labels(content1, title)
        content2 = _namespace_c_variant_labels(content2, title)
    underlining = "=" * len(title)
    header_content = "%s\n\n%s\n%s\n\n" % (label, title, underlining)

    # Concatenate the contents
    combined_content = header_content + "\n" + content1 + "\n\n" + content2  # Adding a newline between contents

    # Generate the filename for the new combined file
    base_filename = os.path.splitext(os.path.basename(file1))[0]  # Extract the base filename without extension
    combined_filename = f"{base_filename}.rst"

    # Create the full path for the new file
    output_path = os.path.join(output_directory, combined_filename)

    # Save the combined content to the new file
    with open(output_path, 'w', encoding='utf-8') as output_file:
        output_file.write(combined_content)

    print(f"Created combined file: {output_path}")
    return combined_filename


def build_rst_structure_leaf_only(root_dir, output_name="index.rst"):
    """
    Generate index.rst with a toctree for each non-leaf directory, including subdirectories.
    Use .rst files only in leaf directories (no subfolders).
    """

    def list_rst_files(directory):
        return sorted([
            f for f in os.listdir(directory)
            if f.endswith(".rst") and f != output_name and not f.startswith("_")
        ])

    def is_leaf_directory(directory):
        return not any(
            os.path.isdir(os.path.join(directory, item)) and not item.startswith(".")
            for item in os.listdir(directory)
        )

    def process_directory(path):
        entries = []
        subdirs = [
            d for d in sorted(os.listdir(path))
            if os.path.isdir(os.path.join(path, d)) and not d.startswith(".")
        ]

        for subdir in subdirs:
            subdir_path = os.path.join(path, subdir)
            if is_leaf_directory(subdir_path):
                # If it's a leaf, include any .rst files it has
                rst_files = list_rst_files(subdir_path)
                for rst in rst_files:
                    entries.append(os.path.join(subdir, rst))
            else:
                # Recurse and create index.rst if needed
                generated_rst = process_directory(subdir_path)
                if generated_rst:
                    entries.append(os.path.join(subdir, generated_rst))

        if entries:
            title = os.path.basename(path).capitalize() or "Root"
            underline = "=" * len(title)

            rst_lines = [title, underline, "", ".. toctree::", "   :maxdepth: 1", ""]
            rst_lines += [f"   {os.path.splitext(entry)[0]}" for entry in entries]

            output_path = os.path.join(path, output_name)
            with open(output_path, "w") as f:
                f.write("\n".join(rst_lines))

            print(f"Created {output_path}")
            return output_name

        return None  # nothing to include

    process_directory(root_dir)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Combine C++ documentation and Python documentation')
    parser.add_argument('-d', '--directories', nargs='+', required=True, help='The directories containing the C++ documentation')
    parser.add_argument('-g', '--generated-cxx-docs', required=True, help='The path to the generated C++ documentation')
    parser.add_argument('-o', '--output', required=True, help='The output directory for the combined documentation')
    parser.add_argument('-e', '--exclude', nargs='+', help='Exclude files or directories matching these patterns')

    args = parser.parse_args()

    directory_paths = args.directories
    generated_cxx_docs_path = args.generated_cxx_docs
    output_directory = args.output
    exclude_patterns = args.exclude

    rst_files = []
    for directory in directory_paths:
        rst_files.extend(collect_rst_files(directory, exclude_patterns))
    dashed_filenames = [file_group.underscore_filename for file_group in rst_files]

    # Append "class" and find the corresponding files
    fill_matching_pairs(generated_cxx_docs_path, rst_files)
    print("rst_files ", rst_files)
    # Concatenate the files for each matched pair and save the result in the specified output directory
    combined_files = []
    sorted_rst_files = sorted(rst_files, key=lambda file_group: file_group.categories)

    os.makedirs(output_directory, exist_ok=True)
    for file_group in sorted_rst_files:
        # create directories
        os.makedirs(os.path.join(output_directory, file_group.categories), exist_ok=True)

    for file_group in sorted_rst_files:

        combined_filename = concatenate_rst_files(file_group.rst_file_path,
                                                  file_group.class_rst_file_path,
                                                  os.path.join(output_directory, file_group.categories))
        # Add the new combined file path to the list
        combined_files.append(os.path.join(output_directory, file_group.categories, combined_filename))

    # Post-process: detect and fix duplicate equation labels across all output files
    # by prefixing with the module name
    eq_label_files = {}  # label -> list of file paths
    for combined_file in combined_files:
        try:
            with open(combined_file, 'r', encoding='utf-8') as f:
                content = f.read()
        except FileNotFoundError:
            continue
        for m in re.finditer(r':label:\s*(eq[^:\s]*:\S+)', content):
            label = m.group(1)
            eq_label_files.setdefault(label, []).append(combined_file)

    # Find duplicates and prefix with module name
    for label, files in eq_label_files.items():
        if len(files) > 1:
            for fpath in files:
                module_name = os.path.splitext(os.path.basename(fpath))[0]
                with open(fpath, 'r', encoding='utf-8') as f:
                    content = f.read()
                new_label = label.replace(':', f'_{module_name}:', 1)
                content = content.replace(f':label: {label}', f':label: {new_label}')
                content = content.replace(f':eq:`{label}`', f':eq:`{new_label}`')
                with open(fpath, 'w', encoding='utf-8') as f:
                    f.write(content)
                print(f"  Renamed duplicate eq label '{label}' -> '{new_label}' in {fpath}")

    build_rst_structure_leaf_only(output_directory)
