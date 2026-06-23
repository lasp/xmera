from os import system, cpu_count

system(f'cd build && ctest -C Release --parallel {cpu_count() or 1}')
system('cd src && pytest -n auto')
