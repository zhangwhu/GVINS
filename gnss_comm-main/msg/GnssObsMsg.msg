# This message contains one-epoch measurements from one single satellite

GnssTimeMsg time    # measurement time
uint8 sat           # satellite ID (define by `sat_no` function in `gnss_utility.hpp`)
uint8 rcv           # satellite ID (define by `sat_no` function in `gnss_utility.hpp`)
uint16[] SNR        # carrier-to-noise density ratio (signal strength) [dB-Hz]
uint8[] LLI         # lost-lock indicator (1=lost)
uint8[] code        # channel code
float64[] psr       # pseudorange measurement [m]
float64[] cp        # carrier phase measurement [cycle]
float32[] dopp      # Doppler measurement [Hz]