# NOTE: can only be run in linux terminal or git bash

# make a bin folder if one doesn't already exist
if ! [ -d bin ]; then
    mkdir bin
fi

echo "Compiling tests"
echo

# compile tests
for FILE in $@; do

    # look for file
    if [ -f $FILE ]; then
        g++ -o "./bin/${FILE/.cpp/.exe}" $FILE
    else
        echo "$FILE not found"
    fi

done

# run tests
for FILE in $@; do

    # look for file
    if [ -f $FILE ]; then
        CLASSNAME=$(echo $FILE | grep -o "[^/]*\." | grep -o "^[^\.]*" | tr -d '\n')

        echo
        echo -e "Testing \e[96m$CLASSNAME\e[0m"
        echo

        ./bin/${FILE/.cpp/.exe}
        
    fi

done