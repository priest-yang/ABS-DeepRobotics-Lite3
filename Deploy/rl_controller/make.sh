if [ "$(basename "$(pwd)")" = "build" ]; then
    rm -rf *
    # cmake -DBUILD_SIM=ON .. -DBUILD_PLATFORM=x86
    cmake -DBUILD_SIM=OFF -DSEND_REMOTE=ON .. -DBUILD_PLATFORM=arm
    make -j4
else
    echo "The current directory is not named 'build'."
fi
                         