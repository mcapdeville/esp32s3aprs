import gc
import esp32s3aprs

def iprint(iter):
    i = 0;
    for o in iter:
        i = i+1
        print(i, o)
        print()
    print(i, "element(s)")

Radio = esp32s3aprs.radio();
Aprs = esp32s3aprs.aprs();
Stations = Aprs.stations();

gc.collect()
print("Micropython heap free :",gc.mem_free())
