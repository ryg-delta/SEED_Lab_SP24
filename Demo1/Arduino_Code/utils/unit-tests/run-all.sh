# NOTE: can only be run in linux terminal or Git Bash

# compile all tests

echo "Compiling tests"
echo ""

for FILE in *.cpp; do 

    g++ -o "./bin/${FILE/.cpp/.exe}" $FILE
    
done

# run all tests

for FILE in ./bin/*.exe; do

    CLASSNAME=$(echo $FILE | grep -o "[^/]*\.exe" | grep -o "^[^\.]*" | tr -d '\n')
    echo -e "Testing \e[96m$CLASSNAME\e[0m"
    echo

    ./$FILE

done