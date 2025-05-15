#!/bin/bash
#source ./runTests.sh

# Function for creating the zip with all required files
create_zip() {

    # The variable declaration and assignment should be separate
    # nombre should be a string, not an integer

    nombre="AI_Practice_2025_100495811_100495713_100495775"
    ext="zip"
    archivo="${nombre}.${ext}"

    # Remove the old zip file if it exists
    rm -f $archivo

    # Make new dirs to copy the files into
    mkdir py
    mkdir py/utils
    mkdir json
    # python code
    cp ../src/py/*.py ./py/
    cp ../src/py/*.txt ./py/
    cp ../src/py/utils/*.py ./py/utils/
    # json code
    cp ../src/json/* ./json/
    # readme file
    cp ../src/README.md .

    # Create the zip file with all necessary files
    zip -r $archivo py/ json/ README.md "$nombre".pdf

    # Clean up temporary files
    rm -rf py
    rm -rf json
    rm README.md
}

create_zip

# Se puede crear el zip habiendo o no ejecutado los tests anteriormente
#read -p "Quieres ejecutar los tests andtes de crear el archivo zip? (y/n)" respuesta
#if [ "$respuesta" == "y" ]; then
#    run_tests
#    if [ $? -eq 0 ]; then
#        echo "EJECUCIÓN DE TESTS: OK"
#        crear_zip
#        exit
#    else
#        echo "EJECUCIÓN DE TESTS: ERROR"
#        echo "No se crea el archivo zip"
#        exit
#    fi
#
#elif [ "$respuesta" == "n" ]; then
#
#    crear_zip
#else
#    echo "Respuesta no válida"
#    exit
#fi
