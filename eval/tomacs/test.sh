while [ $(cat ./t) != 1 ];
do
    echo sleep 1;
    sleep 1;
    cat ./t
    echo 1 > ./t
    cat ./t
    sleep 1
    echo 0 > ./t
done
    
