##################################################
#author: Novak                                   #
#email: novak@gekkoscience.com                   #
#BTC address: 1NovakVK1FWdh9gs41dckVQS8bxSzwiaRw #
##################################################

You may compile compac support with the ./configure option --enable-gekko.
You can set the clock speed between 100 and 500 MHz using the --compac-freq option, do not expect it to run over about 150MHz on stock USB power (depending on voltage and the individual stick).

There is an example file showing usage: cgminer_run.sh
Usage:
./cgminer --compac-freq <num>

<num> is any integer from 100 to 500.

You can calculate the expected hashrate as 0.055*(clock_in_mhz)=GH/s.  For example, 200 MHz gives 11GH/s.

--
novak
