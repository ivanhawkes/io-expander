echo Deploying IO Expander test to the Pico.
script="$0"
basename="${script%/*}"

openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 4000" -c "program ${basename}/build/src/io-expander.elf verify reset exit"

