
%% define the mapping between the Qulog and QuProlog calls
%% This means the a call on
%% tcp_server(Socket, Port, Host)
%% in Qulog
%% code will get compiled into the call
%% tcp_server_interface(Socket, Port, Host)
%% at the the QuProlog level

qulog2qp_map(tcp_server(Socket, Port, Host),
             tcp_server_interface(Socket, Port, Host)).
qulog2qp_wrap(tcp_server).

%% Lifting gmtime - note that gmtime is bidirectional but only
%% one type is allowed for lifted relations and actions and so we
%% need two relations - one for each direction (type).

qulog2qp_map(time2gmtime(A, B), gmtime(A, B)).
qulog2qp_map(gmtime2time(A, B), gmtime(B, A)).


%% For the interface use catch to trap any exceptions generated
%% Note that we convert the socket ID  into a constructor term
%% of the required type
tcp_server_interface(socket(Socket), Port, Host) :-
    catch(tcp_server(Socket, Port, Host), Pattern,
           handle_tcp_exception(Pattern)).

%% convert the QuProlog exception to the corresponding
%% Qulog exception where the argument is a string
%% representation of the QuProlog exception
handle_tcp_exception(Pattern) :-
    open_string(write, Stream), 
    write_term_list(Stream, [Pattern]),
    stream_to_string(Stream, Str),
    throw(tcp_exception(Str)).


