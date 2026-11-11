This directory stores recorded-Episode protocol files named
`<task>_episodeNNNNNNNN.protocol.jsonl` by default.

The first line is metadata. Every later line contains an exact inference request
and a server-shaped response built from one chunk of recorded actions and masks.
Temporary decoded images exist only under `data/cache/` during conversion and
are removed after the protocol file is committed.
