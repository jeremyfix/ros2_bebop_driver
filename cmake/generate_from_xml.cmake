include(FetchContent)

function(generate_states xml_filename)
	message("Generating states from https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/${ARSDKXML_HASH}/xml/${xml_filename}")	

	set(XML_URL https://raw.githubusercontent.com/Parrot-Developers/${ARSDK_HASH}/xml/${xml_filename})

	# Get the xml file
	file(DOWNLOAD ${XML_URL} ${xml_filename})

	#TODO: Generate the files; Will output the generated files

	#TODO: Install the generated files
endfunction()

function(generate_settings xml_filename)
	message("Generating settings from https://raw.githubusercontent.com/Parrot-Developers/arsdk-xml/${ARSDKXML_HASH}/xml/${xml_filename}")	

	set(XML_URL https://raw.githubusercontent.com/Parrot-Developers/${ARSDK_HASH}/xml/${xml_filename})

	# Get the xml file
	file(DOWNLOAD ${XML_URL} ${xml_filename})

	#TODO: Generate the files; Will output the generated files

	#TODO: Install the generated files
endfunction()
