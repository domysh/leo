# CMakeLists.txt for LEO module examples
# Based on the original wscript configuration

# Basic examples that don't require special dependencies
set(basic_examples
    leo-circular-orbit-tracing-example
    leo-car-move-trace-example
)

# Examples that require additional modules (aodv, epidemic-routing)
set(advanced_examples
    leo-delay-tracing-example
    leo-bulk-send-example
    leo-nr-send-example
    
)

# Build basic examples
foreach(
  example
  ${basic_examples}
)
  build_lib_example(
    NAME ${example}
    SOURCE_FILES ${example}.cc
    LIBRARIES_TO_LINK ${libleo}
                      ${libcore}
                      ${libmobility}
  )
endforeach()

# Build advanced examples (check if required modules are available)
if(aodv IN_LIST ns3-all-enabled-modules)
  # Check if epidemic-routing is available
  if(epidemic-routing IN_LIST ns3-all-enabled-modules)
    if (nr IN_LIST ns3-all-enabled-modules)
      foreach(
        example
        ${advanced_examples}
      )
        build_lib_example(
          NAME ${example}
          SOURCE_FILES ${example}.cc
          LIBRARIES_TO_LINK ${libleo}
                            ${libcore}
                            ${libmobility}
                            ${libaodv}
                            ${libepidemic-routing}
        )
      endforeach()
    else()
      message(
        WARNING
          "leo-delay-tracing-example and leo-bulk-send-example require nr (5g-lena) module"
      )
    endif()
  else()
    message(
      WARNING
        "leo-delay-tracing-example and leo-bulk-send-example require epidemic-routing module"
    )
  endif()
else()
  message(
    WARNING
      "leo-delay-tracing-example and leo-bulk-send-example require aodv module"
  )
endif()
