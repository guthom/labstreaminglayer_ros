"""Example program to show how to read a multi-channel time series from LSL."""

from pylsl import StreamInlet, resolve_stream

# first resolve an EEG stream on the lab network
print("looking for an Testing stream...")
streams = []

streams.append(resolve_stream("name", "/floatFromRos"))
streams.append(resolve_stream("name", "/floatFromRos1"))
streams.append(resolve_stream("name", "/floatFromRos2"))
streams.append(resolve_stream("name", "/floatFromRos3"))


# create a new inlet to read from the stream
inlets = []
for stream in streams:
    inlets.append(StreamInlet(stream[0]))

while True:
    # get a new sample (you can also omit the timestamp part if you're not
    # interested in it)
    for inlet in inlets:
        sample, timestamp = inlet.pull_sample(timeout=0)
        if sample and timestamp is not None:
            print(timestamp, sample)