include(FetchContent)

function(generate_states xml_filename)

    set(XML_URL https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/${ARSDKXML_HASH}/xml/${xml_filename})
    message("Generating states from ${XML_URL}")	

	# Get the xml file
	file(DOWNLOAD ${XML_URL} ${xml_filename})

    execute_process(COMMAND python3 ${CMAKE_SOURCE_DIR}/cmake/generate_states.py ${xml_filename}
        ${CMAKE_SOURCE_DIR}/cmake/templates_states)
endfunction()

function(generate_settings xml_filename)
    set(XML_URL https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/${ARSDKXML_HASH}/xml/${xml_filename})
    message("Generating states from ${XML_URL}")	

	# Get the xml file
	file(DOWNLOAD ${XML_URL} ${xml_filename})

    execute_process(COMMAND python3 ${CMAKE_SOURCE_DIR}/cmake/generate_settings.py ${xml_filename}
        ${CMAKE_SOURCE_DIR}/cmake/templates_states)

endfunction()
