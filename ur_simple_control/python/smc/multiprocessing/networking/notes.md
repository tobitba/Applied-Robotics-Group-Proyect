networking in smc
-----------------
as in the rest of the library, the simplest good option was chosen.
in particular, serialization is done with protobuf,
and the communication is done over directly over a socket.
for every new use-case, 
you need to define new messages in protobuf, compile them,
and import the compiled ..._pb2.py file.
compilation is something like

```bash
protoc --proto_path=$(pwd) --python_out=$(pwd) $(pwd)/message_specs.proto
```

note! if you need to interact with old software, you will need
to compile the protobuf with an appropriately old compiler version.
more precisely, new codegen and old runtime can not be combined.
open up python, check google.protobuf.__version__ . the major version need 
to align. go to https://github.com/protocolbuffers/protobuf/releases 
, download the apropriate binary and use that to compile the messages.proto

limitations
-------------
minimal overhead is added for everything to work. 
in particular, multiple the messages can parsed out of a single
recv() because getting 2 messages before reading either can certainly happen.
however, storing a buffer in case a message(s) is longer than 1024 bytes will not
be implemented until a need for it arrives.
this also means that if some many messages are piled up that one of them gets sliced up,
the program breaks with a parsing error.
since the current use case is sending up to 20 doubles at a time,
buffer implementation is postponed.

udp communication is also postponed for the time being
because tcp works on time for the current use case.

server
-------
the server is the sender in this case.
since the client (receiver) is dependent on the sender,
the sender needs to start first and bind to a port - hence
the sender is the server.

starts sending stuff once a client is connected.
only supports one client literally just because there was no point
to write more code for the current use-case.

client
------
waits for messages. keeps latest value stored in a separate variable.
you can ask for the data at any point, and you will be served with the
latest data received (i.e. a copy of the stored variable).


using the server or client
------------------------
