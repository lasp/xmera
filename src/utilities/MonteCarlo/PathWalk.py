# SPDX-License-Identifier: ISC
# Copyright (c) 2025, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
#
"""Walk dotted and indexed attribute paths on a sim instance, and apply dispersion values at them."""

import ast
import re


class DispersionApplyError(Exception):
    """Raised when the controller cannot apply a dispersion modification to the sim instance."""


_PATH_TOKEN_RE = re.compile(r'([a-zA-Z_][a-zA-Z0-9_]*)|\[([^\]]+)\]')


def _tokenize_path(path: str) -> list[tuple[str, object]]:
    """Parse a dotted and indexed path into a list of tokens.

    Each token is ('attr', name) or ('index', value). For example, 'foo.bar[2].baz' gives four tokens.
    A path that contains other syntax, for example a call such as 'get_model().x', raises
    :class:`DispersionApplyError`.
    """
    tokens = []
    position = 0
    for match in _PATH_TOKEN_RE.finditer(path):
        attr, index = match.group(1), match.group(2)
        separator = path[position:match.start()]
        expected_separator = '.' if attr is not None and tokens else ''
        if separator != expected_separator:
            raise DispersionApplyError(f"unparseable path {path!r} at position {position}")
        if attr is not None:
            tokens.append(('attr', attr))
        else:
            if not tokens:
                raise DispersionApplyError(f"path {path!r} cannot start with an index")
            try:
                tokens.append(('index', ast.literal_eval(index)))
            except (ValueError, SyntaxError) as e:
                raise DispersionApplyError(f"index {index!r} in path {path!r} is not a literal") from e
        position = match.end()
    if not tokens:
        raise DispersionApplyError(f"empty or unparseable path: {path!r}")
    if position != len(path):
        raise DispersionApplyError(f"unparseable path {path!r} at position {position}")
    return tokens


def _resolve_parent(obj, tokens):
    current = obj
    for token in tokens[:-1]:
        try:
            current = getattr(current, token[1]) if token[0] == 'attr' else current[token[1]]
        except (AttributeError, IndexError, KeyError, TypeError) as e:
            raise DispersionApplyError(f"failed to resolve segment {token!r}") from e
    return current, tokens[-1]


def resolve_path(obj, path: str):
    """Return the object at ``path`` on ``obj``.

    Raises :class:`DispersionApplyError` if the path cannot be parsed or a segment does not exist.
    """
    tokens = _tokenize_path(path)
    parent, leaf = _resolve_parent(obj, tokens)
    return _get_leaf(parent, leaf)


def _get_leaf(parent, leaf):
    try:
        return getattr(parent, leaf[1]) if leaf[0] == 'attr' else parent[leaf[1]]
    except (AttributeError, IndexError, KeyError, TypeError) as e:
        raise DispersionApplyError(f"failed to access leaf {leaf!r}") from e


def _set_leaf(parent, leaf, value) -> None:
    if leaf[0] == 'attr':
        setattr(parent, leaf[1], value)
    else:
        parent[leaf[1]] = value


def apply_modification(sim_instance, path: str, value_str: str) -> None:
    """Apply one dispersion modification to ``sim_instance`` at ``path``.

    The function follows ``path`` to its leaf. If the leaf is data, the function sets the leaf to the
    parsed value. If the leaf is callable, the function calls the leaf with the parsed value.
    ``value_str`` must be a Python literal expression that :func:`ast.literal_eval` can parse.
    """
    tokens = _tokenize_path(path)
    parent, leaf = _resolve_parent(sim_instance, tokens)
    try:
        parsed_value = ast.literal_eval(value_str)
    except (ValueError, SyntaxError) as e:
        raise DispersionApplyError(f"cannot parse value {value_str!r} for path {path!r}") from e
    target = _get_leaf(parent, leaf)
    if callable(target):
        args = parsed_value if isinstance(parsed_value, tuple) else (parsed_value,)
        target(*args)
    else:
        _set_leaf(parent, leaf, parsed_value)


def apply_modifications(sim_instance, modifications: dict) -> None:
    """Apply each ``path: value_str`` item of ``modifications`` to ``sim_instance``, in order."""
    for path, value_str in modifications.items():
        apply_modification(sim_instance, path, value_str)


def generate_modifications(sim_instance, dispersions, modifications=None, magnitudes=None) -> dict:
    """Generate one value for each dispersion, as a ``path: value_str`` dictionary.

    A path that is already in ``modifications`` keeps its value. Thus a rerun uses the saved values.
    A dispersion that does not accept ``get_name()`` without an index is co-dependent: the function
    calls ``generate`` one time, and then reads each of its ``number_of_sub_disps`` values.

    :param sim_instance: The sim that the dispersions read their nominal values from.
    :param dispersions: The dispersion objects.
    :param modifications: The dictionary to add to. If None, the function makes a new dictionary.
    :param magnitudes: If not None, the function adds the magnitude string of each new value.
    :return: The modifications dictionary.
    """
    if modifications is None:
        modifications = {}
    for disp in dispersions:
        try:
            name = disp.get_name()
            if name not in modifications:
                modifications[name] = disp.generate_string(sim_instance)
                if magnitudes is not None:
                    magnitudes[name] = disp.generate_mag_string()
        except TypeError:
            disp.generate(sim_instance)
            for i in range(1, disp.number_of_sub_disps + 1):
                name = disp.get_name(i)
                if name not in modifications:
                    modifications[name] = disp.generate_string(i, sim_instance)
                    if magnitudes is not None:
                        magnitudes[name] = disp.generate_mag_string()
    return modifications


def apply_dispersions(sim_instance, dispersions) -> dict:
    """Generate a value for each dispersion and apply it to ``sim_instance``.

    This is the same generate and apply sequence that a Monte Carlo worker uses, without the random
    seeds. Use it to test that dispersion paths resolve and values reach the sim.

    :return: The applied ``path: value_str`` dictionary.
    """
    modifications = generate_modifications(sim_instance, dispersions)
    apply_modifications(sim_instance, modifications)
    return modifications
