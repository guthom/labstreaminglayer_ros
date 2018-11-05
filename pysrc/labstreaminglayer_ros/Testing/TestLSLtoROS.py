"""Example program to show how to read a multi-channel time series from LSL."""

from pylsl import StreamOutlet, StreamInfo

# first resolve an EEG stream on the lab network
print("looking for an Testing stream...")
outlets = []

streamInfo = StreamInfo(name="/floatFromlsl", type="Testing",channel_count=1, nominal_srate=30,
                                        channel_format="float32")
outlets.append(StreamOutlet(streamInfo))


value = 0.0
while True:
    # get a new sample (you can also omit the timestamp part if you're not
    # interested in it)
    for outlet in outlets:
        print "pushing " + str(value)
        outlet.push_sample([value])

    value += 0.0001