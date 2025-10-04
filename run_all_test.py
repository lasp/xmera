from os import system

system('cd build && ctest -C Release')
system('cd src && pytest -n auto')
