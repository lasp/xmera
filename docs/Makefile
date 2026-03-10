# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
# ifeq ($(OS),Windows_NT)
#     # windows specific code
#     SPHINXBUILD = py -3 -m sphinx
# else
#     # unix/macOS specific code
#     SPHINXBUILD = python -m sphinx
# endif

ROOT 		 ?=
SPHINXBUILD   = sphinx-build
PAPER         =
BUILDDIR     ?= build
SOURCEDIR    ?= source
# DEBUG        ?= -P
# SPHINXOPTS   ?= -v -W -E
SPHINXOPTS   ?= -v -E
# -n triggers an additional 100+ warnings (all about undefined references) that -W treats as errors.
# these extraneous warnings are ignored because some doc pages have to avoid duplicate IDs
ALLSPHINXOPTS = $(DEBUG) -d $(BUILDDIR)/doctrees $(SPHINXOPTS) source

# User-friendly check for sphinx-build
ifeq ($(shell which $(SPHINXBUILD) >/dev/null 2>&1; echo $$?), 1)
$(error The '$(SPHINXBUILD)' command was not found. Make sure you have Sphinx installed, then set the SPHINXBUILD environment variable to point to the full path of the '$(SPHINXBUILD)' executable. Alternatively you can add the directory with the executable to your PATH. If you don't have Sphinx installed, grab it from http://sphinx-doc.org/)
endif

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

clean :
	-rm -rf $(BUILDDIR)
	-rm -rf $(SOURCEDIR)/Documentation
	-rm -rf $(SOURCEDIR)/externalTools
	-rm -rf $(SOURCEDIR)/examples
	-rm -rf $(SOURCEDIR)/api/msgPayloads
	-rm -rf $(SOURCEDIR)/api/baseClasses
	-rm -rf $(SOURCEDIR)/api/modules
	-rm -rf generated_cxx_docs

html:
	doxygen Doxyfile.in
	python apidoc.py "./doxygen_output_xml" --members --project "xmera" --output-dir "./generated_cxx_docs"
	python collect_module_docs.py --directories '../src/simulation' '../src/fswAlgorithms' \
	--exclude '_GeneralModuleFiles' '_UnitTest' --generated-cxx-docs './generated_cxx_docs/class' \
	--output './source/api/modules'
	python generate_msg_payload_rst.py
	python generate_base_class_rst.py
	python generate_examples_rst.py '../examples' './source/examples'
	$(SPHINXBUILD) -b html $(ALLSPHINXOPTS) $(BUILDDIR)/html
	@echo
	@echo "Build finished. The HTML pages are in $(BUILDDIR)/html."

view:
	    # works on macOS
		open build/html/index.html

live:
	sphinx-autobuild $(SOURCEDIR) $(BUILDDIR)

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
