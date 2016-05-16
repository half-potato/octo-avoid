FILE(REMOVE_RECURSE
  "CMakeFiles/example.dir/src/simple_example.cpp.o"
  "devel/lib/octo_avoid/example.pdb"
  "devel/lib/octo_avoid/example"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/example.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
