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


def _apply_modification(sim_instance, path: str, value_str: str) -> None:
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
