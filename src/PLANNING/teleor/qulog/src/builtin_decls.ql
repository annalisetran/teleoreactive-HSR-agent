'$builtin_default_args'(get_line(_12815), get_line(_12815, stdin)).
'$builtin_default_args'(put_line(_12815), put_line(_12815, stdout)).
'$builtin_default_args'(read_term(_12815), read_term(_12815, stdin)).
'$builtin_default_args'(read_op_term(_12815), read_op_term(_12815, stdin)).
'$builtin_default_args'(peek(_12815), peek(_12815, stdin)).
'$builtin_default_args'(flush_output('$none_'), flush_output(stdout)).
'$builtin_default_args'(write_list(_12815), write_list(_12815, stdout)).
'$builtin_default_args'(write_op_term(_12815), write_op_term(_12815, stdout)).
'$builtin_default_args'(fork(_12816, _12818), fork(_12816, _12818, [])).
'$builtin_default_args'(connect_to_pedro('$none_'), connect_to_pedro(localhost, 4550)).
'$builtin_default_args'(connect_to_pedro(_12815), connect_to_pedro(_12815, 4550)).
'$builtin_default_args'(peek_messages(_12816, _12818), peek_messages(_12816, _12818, -1)).
'$builtin_default_args'(range(_12817, _12819, _12821), range(_12817, _12819, _12821, 1)).
'$builtin_default_args'(start_task(_12816, _12818), start_task(_12816, _12818, task)).
'$builtin_type_descr'(stream_num_type_, '$constr_enum_type'([stream_(nat)]), defined_type, "").
'$builtin_type_descr'(stream_enum_type_, '$enum_type'([stdin, stdout, stderr]), defined_type, "").
'$builtin_type_descr'(stream_type, '$union_type'([stream_enum_type_, stream_num_type_]), macro_type, "Used for stream related actions eg open/close/read/write").
'$builtin_type_descr'(open, open(:('$$var$$'('FileName'), !(atom)), :('$$var$$'('RW'), !(atom)), :('$$var$$'('Stream'), ?(stream_type))), act, "Open the file FileName in RW mode, matching Stream with generated stream").
'$builtin_type_descr'(close, close(:('$$var$$'('Stream'), !(stream_type))), act, "Close the stream Stream").
'$builtin_type_descr'(write_type_nl__, '$enum_type'([nl_]), defined_type, "The atom that indicates a newline in argument list\nof the write_list action").
'$builtin_type_descr'(write_type_other__, '$constr_enum_type'([sp_(int), q_(string), uq_(atom), wr_(term)]), defined_type, "The compound layout terms that can be used in argument\nlist of the write_list action").
'$builtin_type_descr'(write_type__, '$union_type'([write_type_nl__, write_type_other__]), macro_type, "The union type of all layout terms for use in  \nargument list of the write_list action").
'$builtin_type_descr'(get_line, get_line(:('$$var$$'('Line'), ?(string)), default(:('$$var$$'('Stream'), !(stream_type)), stdin)), act, "Read Line from Stream").
'$builtin_type_descr'(put_line, put_line(:('$$var$$'('Line'), !(string)), default(:('$$var$$'('Stream'), !(stream_type)), stdout)), act, "Write Line to Stream").
'$builtin_type_descr'(read_term, read_term(??(term), default(:('$$var$$'('Stream'), !(stream_type)), stdin)), act, "Unifies its argument with the next term denoted by the next\nsequence of characters in the stream followed by fullstop, return.").
'$builtin_type_descr'(read_op_term, read_op_term(??(term), default(:('$$var$$'('Stream'), !(stream_type)), stdin)), act, "Unifies its argument with the next term denoted by the next\nsequence of characters in the stream followed by fullstop, return. The\nread uses QuProlog level op declarations.").
'$builtin_type_descr'(peek, peek(?(atom), default(:('$$var$$'('Stream'), !(stream_type)), stdin)), act, "Peek at the next character (as an atom) in Stream. \nEOF is the empty atom ''").
'$builtin_type_descr'(flush_output, flush_output(default(:('$$var$$'('Stream'), !(stream_type)), stdout)), act, "Flush Stream").
'$builtin_type_descr'(term2string, term2string(:('$$var$$'('Term'), ??(term)), :('$$var$$'('String'), ?(string))), rel, "Term is converted to String - the string representation of the term. \nTerm is left unchanged.").
'$builtin_type_descr'(str, ->(str(term), string), fun, "Return the string representation of the (ground) term.").
'$builtin_type_descr'(string2term, string2term(:('$$var$$'('String'), !(string)), :('$$var$$'('Term'), ??(term))), rel, "String is a string comprising the character sequence of a QuLog term. \nTerm is unified with that term.").
'$builtin_type_descr'(write_list, write_list(:('$$var$$'('TermList'), !(list(??(term)))), default(:('$$var$$'('Stream'), !(stream_type)), stdout)), act, "Write TermList to Stream leaving TermList unchanged.").
'$builtin_type_descr'(write_list_to_string, write_list_to_string(:('$$var$$'('TermList'), !(list(??(term)))), :('$$var$$'('String'), ?(string))), rel, "Write TermList to string String leaving TermList unchanged.").
'$builtin_type_descr'(write_op_term, write_op_term(:('$$var$$'('Term'), ??(term)), default(:('$$var$$'('Stream'), !(stream_type)), stdout)), act, "Write Term to Stream. The write uses QuProlog level op declarations. \nTerm is left unchanged.").
'$builtin_type_descr'(log_list, log_list(!(list(??(term)))), act, "Display a list of terms to the TR logger with layout controlled \nusing write_type_ descriptors. Term is left unchanged.").
'$builtin_type_descr'(writeDebug, writeDebug(!(list(??(term)))), rel, "A relational version of write_list - meant to be only used for \ndebugging purposes. Term is left unchanged.").
'$builtin_type_descr'(*, ->(*(nat, nat), nat), fun, "Infix multiplication operator - left associative").
'$builtin_type_descr'(*, ->(*(int, int), int), fun, "Infix multiplication operator - left associative").
'$builtin_type_descr'(*, ->(*(num, num), num), fun, "Infix multiplication operator - left associative").
'$builtin_type_descr'(**, ->(**(nat, nat), nat), fun, "Infix exponentiation operator - not associative").
'$builtin_type_descr'(**, ->(**(int, int), int), fun, "Infix exponentiation operator - not associative").
'$builtin_type_descr'(**, ->(**(num, num), num), fun, "Infix exponentiation operator - not associative").
'$builtin_type_descr'(+, ->(+(nat, nat), nat), fun, "Infix addition operator - left associative").
'$builtin_type_descr'(+, ->(+(int, int), int), fun, "Infix addition operator - left associative").
'$builtin_type_descr'(+, ->(+(num, num), num), fun, "Infix addition operator - left associative").
'$builtin_type_descr'(-, ->(-(int, int), int), fun, "Infix subtraction operator - left associative\nand number complement prefix operator").
'$builtin_type_descr'(-, ->(-(num, num), num), fun, "Infix subtraction operator - left associative\nand number complement prefix operator").
'$builtin_type_descr'(/, ->(/(num, num), num), fun, "Infix division operator - left associative\nNote it always returns a floating point number\nUse // for integer division returning an integer").
'$builtin_type_descr'(//, ->(//(nat, nat), nat), fun, "Infix integer division operator - left associative\nReturns integer part of the division operation").
'$builtin_type_descr'(//, ->(//(int, int), int), fun, "Infix integer division operator - left associative\nReturns integer part of the division operation").
'$builtin_type_descr'(/\, ->(/\(int, int), int), fun, "Infix, integer bitwise AND operator - left associative").
'$builtin_type_descr'(>>, ->(>>(int, int), int), fun, "Shift bits of integer first arg. right by pos integer of second").
'$builtin_type_descr'(\, ->(\(int), int), fun, "Prefix, returns integer that is the bitwise complement\nof the argument integer").
'$builtin_type_descr'(\/, ->(\/(int, int), int), fun, "Infix integer bitwise OR operator - left associative").
'$builtin_type_descr'(abs, ->(abs(int), nat), fun, "The absolute value of number arg.").
'$builtin_type_descr'(abs, ->(abs(num), num), fun, "The absolute value of number arg.").
'$builtin_type_descr'(acos, ->(acos(num), num), fun, "Inverse cosine function").
'$builtin_type_descr'(asin, ->(asin(num), num), fun, "Inverse sine function").
'$builtin_type_descr'(atan, ->(atan(num), num), fun, "Inverse tangent function").
'$builtin_type_descr'(atan2, ->(atan2(:('$$var$$'('Y'), num), :('$$var$$'('X'), num)), num), fun, "The angle in radians between the\npositive x-axis of a plane and the point  (X, Y).").
'$builtin_type_descr'(ceiling, ->(ceiling(num), int), fun, "The smallest integer greater than number arg.").
'$builtin_type_descr'(cos, ->(cos(num), num), fun, "The cosine of radian value argument").
'$builtin_type_descr'(e, ->(e('$none_'), num), fun, "The value of arithmentic constant 'e'").
'$builtin_type_descr'(floor, ->(floor(num), int), fun, "The greatest integer less than the number arg.").
'$builtin_type_descr'(log, ->(log(num), num), fun, "The natural log value.").
'$builtin_type_descr'(max, ->(max(nat, nat), nat), fun, "The maximum of the two numbers").
'$builtin_type_descr'(max, ->(max(int, int), int), fun, "The maximum of the two numbers").
'$builtin_type_descr'(max, ->(max(num, num), num), fun, "The maximum of the two numbers").
'$builtin_type_descr'(min, ->(min(nat, nat), nat), fun, "The minimum of the two numbers").
'$builtin_type_descr'(min, ->(min(int, int), int), fun, "The minimum of the two numbers").
'$builtin_type_descr'(min, ->(min(num, num), num), fun, "The minimum of the two numbers").
'$builtin_type_descr'(mod, ->(mod(:('$$var$$'('M'), int), :('$$var$$'('N'), int)), int), fun, "The remainder of integer division of M by N.\nCan be used as infix operator.").
'$builtin_type_descr'(pi, ->(pi('$none_'), num), fun, "The value of arithmentic constant 'pi'").
'$builtin_type_descr'(prod, ->(prod(list(int)), int), fun, "Returns product of nums or ints on list argument.").
'$builtin_type_descr'(prod, ->(prod(list(num)), num), fun, "Returns product of nums or ints on list argument.").
'$builtin_type_descr'(round, ->(round(num), int), fun, "The nearest integer to number argument.").
'$builtin_type_descr'(sin, ->(sin(num), num), fun, "The sine of radian value argument").
'$builtin_type_descr'(sqrt, ->(sqrt(num), num), fun, "The square root of argument.").
'$builtin_type_descr'(random_int, ->(random_int(:('$$var$$'('M'), int), :('$$var$$'('N'), int)), int), fun, "A random integer between M and N inclusive.").
'$builtin_type_descr'(random_num, ->(random_num('$none_'), num), fun, "A random number between 0 and 1 inclusive.").
'$builtin_type_descr'(random_seed, random_seed(:('$$var$$'('S'), ?(nat))), act, "S is the seed for the random number generator.").
'$builtin_type_descr'(sum, ->(sum(list(int)), int), fun, "The sum of nums or ints on list argument.").
'$builtin_type_descr'(sum, ->(sum(list(num)), num), fun, "The sum of nums or ints on list argument.").
'$builtin_type_descr'(tan, ->(tan(num), num), fun, "The tangent of radian value argument").
'$builtin_type_descr'(<<, ->(<<(int, int), int), fun, "Shift bits of integer first arg. left by pos integer of second").
'$builtin_type_descr'(-, ->(-(int), int), fun, "").
'$builtin_type_descr'(-, ->(-(num), num), fun, "").
'$builtin_type_descr'(=, =(??(term), ??(term)), rel, "Any terms, unification with occurs check").
'$builtin_type_descr'(=@, =@(?('$$var$$'('T')), !('$$var$$'('T'))), rel, "One sided unification").
'$builtin_type_descr'(=@, =@(??('$$var$$'('T')), ??('$$var$$'('T'))), rel, "One sided unification").
'$builtin_type_descr'(<, <(num, num), rel, "Numeric values, less than").
'$builtin_type_descr'(=<, =<(num, num), rel, "Numeric values, less than or equal").
'$builtin_type_descr'(>, >(num, num), rel, "Numeric values, greater than").
'$builtin_type_descr'(>=, >=(num, num), rel, "Numeric values, greater than or equal").
'$builtin_type_descr'(@<, @<(??(term), ??(term)), rel, "QuLog term ordering, less than. \nBoth arguments are left unchanged.").
'$builtin_type_descr'(@=<, @=<(??(term), ??(term)), rel, "QuLog term ordering, less than or equal. \nBoth arguments are left unchanged.").
'$builtin_type_descr'(@>, @>(??(term), ??(term)), rel, "QuLog term ordering, greater than. \nBoth arguments are left unchanged.").
'$builtin_type_descr'(@>=, @>=(??(term), ??(term)), rel, "QuLog term ordering, greater than or equal.\nBoth arguments are left unchanged.").
'$builtin_type_descr'(\=, \=(??(term), ??(term)), rel, "Any terms, the negation of unification with occurs checkl. \nBoth arguments are left unchanged.").
'$builtin_type_descr'(isa, isa(?(term), !(typeE('$$var$$'('_')))), rel, "Can be used to check or generate instances of the type\ndescribed by second argument providing this has a finite number of\nground instances.").
'$builtin_type_descr'(template, template(??(term)), rel, "Tests that its term argument is a compound term with non-variable\nconstructor. The argument is left unchanged.").
'$builtin_type_descr'(var, var(??(term)), rel, "Tests that its term argument is a variable. The argument is left unchanged.").
'$builtin_type_descr'(type, type(??(term), !(typeE('$$var$$'('T')))), rel, "Can be used to check if a value has the type described by type \nthe expression second argument. As a top level QuLog query \ncan be used to find the type of a value. The first argument\nis left unchanged.").
'$builtin_type_descr'(allowed_dyn_call, allowed_dyn_call(??(term)), rel, "Check that the argument is an allowed relation call to a dynamic relation. \nTypically used just before a call on forget. The argument is left unchanged.").
'$builtin_type_descr'(allowed_rel_call, allowed_rel_call(??(term)), rel, "Check that the argument is an allowed relation call, i.e. it is a term\nrepresenting a relation call with the correct modes and types. \nTypically used just before a call call(C). The argument is left unchanged.").
'$builtin_type_descr'(allowed_act_call, allowed_act_call(??(term)), rel, "Check that the argument is an allowed action call, i.e. it is a term\nrepresenting an action call with the correct modes and types. \nTypically used just before a call do(C). The argument is left unchanged.").
'$builtin_type_descr'(type_kind_, '$constr_enum_type'([enumT_(list(atomic)), constrT_(list(term)), unionT_(list(term)), tupleT_(list(term)), rangeT_(int, int)]), defined_type, "").
'$builtin_type_descr'(type_info, type_info(!(typeE('$$var$$'('T'))), ?(type_kind_)), rel, "").
'$builtin_type_descr'(#, ->(#(list(term)), nat), fun, "Operator that returns the length of a list or string or the size of a set").
'$builtin_type_descr'(#, ->(#(set(term)), nat), fun, "Operator that returns the length of a list or string or the size of a set").
'$builtin_type_descr'(#, ->(#(string), nat), fun, "Operator that returns the length of a list or string or the size of a set").
'$builtin_type_descr'(in, in(?('$$var$$'('T')), !(list(!('$$var$$'('T'))))), rel, "Multi-purpose infix membership operator for retrieving or testing\nmembership of lists, sets and strings. For strings the first\nargument is (or will be) a singleton string.").
'$builtin_type_descr'(in, in(??('$$var$$'('T')), !(list(??('$$var$$'('T'))))), rel, "Multi-purpose infix membership operator for retrieving or testing\nmembership of lists, sets and strings. For strings the first\nargument is (or will be) a singleton string.").
'$builtin_type_descr'(in, in(?('$$var$$'('T')), !(set('$$var$$'('T')))), rel, "Multi-purpose infix membership operator for retrieving or testing\nmembership of lists, sets and strings. For strings the first\nargument is (or will be) a singleton string.").
'$builtin_type_descr'(in, in(?(string), !(string)), rel, "Multi-purpose infix membership operator for retrieving or testing\nmembership of lists, sets and strings. For strings the first\nargument is (or will be) a singleton string.").
'$builtin_type_descr'(tolist, ->(tolist(string), list(string)), fun, "Converts a string or set to a list preserving order. For a string the \nresult is the list of characters of the string as singleton strings.").
'$builtin_type_descr'(tolist, ->(tolist(set('$$var$$'('T'))), list('$$var$$'('T'))), fun, "Converts a string or set to a list preserving order. For a string the \nresult is the list of characters of the string as singleton strings.").
'$builtin_type_descr'(toset, ->(toset(string), set(string)), fun, "Converts a string or list to a set. For a string the result\nis the set of characters of the string as singleton strings.").
'$builtin_type_descr'(toset, ->(toset(list('$$var$$'('T'))), set('$$var$$'('T'))), fun, "Converts a string or list to a set. For a string the result\nis the set of characters of the string as singleton strings.").
'$builtin_type_descr'(<>, ->(<>(list('$$var$$'('T')), list('$$var$$'('T'))), list('$$var$$'('T'))), fun, "Infix left associative operator which is both a function for appending \na pair of complete lists and a pattern operator for use\non RHS of =? for non-deterministically splitting a list.").
'$builtin_type_descr'(append, append(!(list('$$var$$'('T'))), !(list('$$var$$'('T'))), ?(list('$$var$$'('T')))), rel, "Similar to the <> operator except it can be used\nwith non-ground lists and lists that have ,..Var undetermined tails").
'$builtin_type_descr'(append, append(?(list('$$var$$'('T'))), ?(list('$$var$$'('T'))), !(list('$$var$$'('T')))), rel, "Similar to the <> operator except it can be used\nwith non-ground lists and lists that have ,..Var undetermined tails").
'$builtin_type_descr'(append, append(!(list(??('$$var$$'('T')))), !(list(??('$$var$$'('T')))), ?(list(??('$$var$$'('T'))))), rel, "Similar to the <> operator except it can be used\nwith non-ground lists and lists that have ,..Var undetermined tails").
'$builtin_type_descr'(append, append(?(list(??('$$var$$'('T')))), ?(list(??('$$var$$'('T')))), !(list(??('$$var$$'('T'))))), rel, "Similar to the <> operator except it can be used\nwith non-ground lists and lists that have ,..Var undetermined tails").
'$builtin_type_descr'(append, append(?(list(??('$$var$$'('T')))), ??(list('$$var$$'('T'))), ??(list('$$var$$'('T')))), rel, "Similar to the <> operator except it can be used\nwith non-ground lists and lists that have ,..Var undetermined tails").
'$builtin_type_descr'(delete, delete(:('$$var$$'('X'), ?('$$var$$'('T'))), :('$$var$$'('Xs'), list('$$var$$'('T'))), :('$$var$$'('R'), ?(list('$$var$$'('T'))))), rel, "R is Xs with X deleted.").
'$builtin_type_descr'(member, member(?('$$var$$'('T')), !(list('$$var$$'('T')))), rel, "Similar to the 'in' operator except the second argument\nmay be a list template, even an unbound variable.").
'$builtin_type_descr'(member, member(??('$$var$$'('T')), ??(list('$$var$$'('T')))), rel, "Similar to the 'in' operator except the second argument\nmay be a list template, even an unbound variable.").
'$builtin_type_descr'(reverse, reverse(!(list('$$var$$'('T'))), ?(list('$$var$$'('T')))), rel, "Can be used to reverse a complete list of template\nterms.  Second argument can be list pattern using ,..").
'$builtin_type_descr'(reverse, reverse(?(list('$$var$$'('T'))), !(list('$$var$$'('T')))), rel, "Can be used to reverse a complete list of template\nterms.  Second argument can be list pattern using ,..").
'$builtin_type_descr'(reverse, reverse(!(list(??('$$var$$'('T')))), ?(list(??('$$var$$'('T'))))), rel, "Can be used to reverse a complete list of template\nterms.  Second argument can be list pattern using ,..").
'$builtin_type_descr'(reverse, reverse(?(list(??('$$var$$'('T')))), !(list(??('$$var$$'('T'))))), rel, "Can be used to reverse a complete list of template\nterms.  Second argument can be list pattern using ,..").
'$builtin_type_descr'(sort, sort(!(list(!('$$var$$'('T')))), ?(list('$$var$$'('T'))), !(rel('$tuple_type'([!('$$var$$'('T')), !('$$var$$'('T'))])))), rel, "Will match the second argument against the first argument sorted by the\ntransitive order relation given as the third argument, without instantiating \nvariables in the first argument. \nIf the relation is asymmetric, duplicate terms will be removed.").
'$builtin_type_descr'(sort, sort(!(list(??('$$var$$'('T')))), ?(list(??('$$var$$'('T')))), !(rel('$tuple_type'([??('$$var$$'('T')), ??('$$var$$'('T'))])))), rel, "Will match the second argument against the first argument sorted by the\ntransitive order relation given as the third argument, without instantiating \nvariables in the first argument. \nIf the relation is asymmetric, duplicate terms will be removed.").
'$builtin_type_descr'(transform_subterms, transform_subterms(rel('$tuple_type'([??(term), ??(term)])), ??(term), ??(term)), rel, "A lift of QuProlog's transform_subterms. The call fails if \nthe transformed term is not of type term.").
'$builtin_type_descr'(collect_simple_terms, collect_simple_terms(rel('$tuple_type'([??(term), ??(term), ??(term)])), ??(term), ??(term), ??(term)), rel, "A lift of QuProlog's collect_simple_terms").
'$builtin_type_descr'(term2list, term2list(!(term), ?(list(term))), rel, "A lift of =.. where the call fails when a generated compound \nterm is not of type term. If a function application is constructed \nthen that application is evaluated.").
'$builtin_type_descr'(term2list, term2list(?(term), !(list(term))), rel, "A lift of =.. where the call fails when a generated compound \nterm is not of type term. If a function application is constructed \nthen that application is evaluated.").
'$builtin_type_descr'(term2list, term2list(??(term), ??(list(term))), rel, "A lift of =.. where the call fails when a generated compound \nterm is not of type term. If a function application is constructed \nthen that application is evaluated.").
'$builtin_type_descr'(diff, ->(diff(set('$$var$$'('T')), set('$$var$$'('T'))), set('$$var$$'('T'))), fun, "Infix operator for finding elements of first set arg. not in the second.").
'$builtin_type_descr'(inter, ->(inter(set('$$var$$'('T')), set('$$var$$'('T'))), set('$$var$$'('T'))), fun, "Infix operator for finding intersection of two sets.").
'$builtin_type_descr'(union, ->(union(set('$$var$$'('T')), set('$$var$$'('T'))), set('$$var$$'('T'))), fun, "Infix operator for finding union of two sets.").
'$builtin_type_descr'(++, ->(++(string, string), string), fun, "Infix left associative operator which is both a function for \nconcatenating a pair of strings and a pattern operator for use on\nLHS of =? for non-deterministically splitting strings.").
'$builtin_type_descr'(sub_string, sub_string(:('$$var$$'('String'), !(string)), :('$$var$$'('Start'), ?(nat)), :('$$var$$'('Length'), ?(int)), :('$$var$$'('After'), ?(nat)), :('$$var$$'('SubString'), ?(string))), rel, "Substring is the sub-string of String of length Length\nstarting at position Start. After is the number of characters\nremaining in the string after Substring.").
'$builtin_type_descr'(re_match, re_match(:('$$var$$'('RE'), !(string)), :('$$var$$'('String'), !(string)), :('$$var$$'('Match'), ?(list('$tuple_type'([nat, nat]))))), rel, "RE is a regular expression, String is the string to match the RE against. \nIf a match is found, Match is a list of \npairs representing the start and end of matches.\nOn backtracking, Match is instantiated to the next list of matches.\nre_match fails if RE is an invalid regular expression.").
'$builtin_type_descr'(map, ->(map(:('$$var$$'('F'), ->('$tuple_type'(['$$var$$'('T1')]), '$$var$$'('T2'))), :('$$var$$'('List'), list('$$var$$'('T1')))), list('$$var$$'('T2'))), fun, "Apply F to each element of List").
'$builtin_type_descr'(filter, ->(filter(:('$$var$$'('Test'), rel('$tuple_type'(['$$var$$'('T1')]))), :('$$var$$'('List'), list('$$var$$'('T1')))), list('$$var$$'('T1'))), fun, "Filter the elements of List using Test").
'$builtin_type_descr'(filter_map, ->(filter_map(:('$$var$$'('Test'), rel('$tuple_type'(['$$var$$'('T1')]))), :('$$var$$'('F'), ->('$tuple_type'(['$$var$$'('T1')]), '$$var$$'('T2'))), :('$$var$$'('List'), list('$$var$$'('T1')))), list('$$var$$'('T2'))), fun, "Filter the elements of List using Test and then \napply F to the filtered elements").
'$builtin_type_descr'(builtin_exception, '$constr_enum_type'([read_term_syntax_error('$none_'), input_term_type_error(atom, string), read_op_term_syntax_error(string), input_term_error(atom, string), string2term_syntax_error(string), stream_not_writeable(string), stream_not_readable(string), stream_not_open(string), prolog_call_type_error(string), action_failure(string), no_matching_action_rule(string), no_matching_function_rule(string), no_matching_tr_rule(tel_term), arithmetic_function_domain_error(term), remote_query_ontology_mismatch(string), remote_query_exception(exception), pedro_not_connected(atom), pedro_name_already_registered(atom), fork_thread_name_exists(atom), qp_exception(string), qulog2qp_type_exception(string), no_agent_to_kill('$none_'), no_matching_task_to_kill(atom), not_start_task_TR_procedure(string), task_name_in_use(atom)]), defined_type, "Constructor type for all qulog exception terms").
'$builtin_type_descr'(exception, '$union_type'([builtin_exception, user_exception]), macro_type, "The type for all exception terms including user exceptions. \nIf the user doesn't define user_exception_ then the system declares it as\ndef user_exception ::= default_exception_().\nThis will be overwritten by a user declaration.").
'$builtin_type_descr'(raise, raise(:('$$var$$'('Patt'), !(exception))), act, "Raise Patt as an exception to be caught in some outer try-except").
'$builtin_type_descr'(thread_sizes_t, '$constr_enum_type'([choice_size(nat), env_size(nat), heap_size(nat), binding_trail_size(nat), other_trail_size(nat), scratchpad_size(nat), name_table_size(nat), ip_table_size(nat)]), defined_type, "Sizes structures for specifying the sizes of data areas when forking.").
'$builtin_type_descr'(exit, exit('$none_'), act, "Exit this thread.").
'$builtin_type_descr'(exit_thread, exit_thread(!(atom)), act, "Exit thread with given name (if it exists).").
'$builtin_type_descr'(fork, fork(:('$$var$$'('Act'), !(act_term)), :('$$var$$'('Name'), ?(atom)), default(:('$$var$$'('Sizes'), !(list(!(thread_sizes_t)))), [])), act, "Creates a new thread named Name executing\nthe action call Act. Name must be new thread name if given,\nthe sizes of the data areas are specified in Size.").
'$builtin_type_descr'(fork_light, fork_light(:('$$var$$'('Act'), !(act_term)), :('$$var$$'('Name'), ?(atom))), act, "Same as fork except the sizes of the data areas are small").
'$builtin_type_descr'(thread_sleep, thread_sleep(!(num)), act, "Causes the executing thread to suspend for given number of seconds.").
'$builtin_type_descr'(is_thread, is_thread(:('$$var$$'('Th'), !(atom))), rel, "Succeeds iff Th is the name of an existing thread.").
'$builtin_type_descr'(agent_handle, '$union_type'([atom, process_handle]), macro_type, "Address formats for agents").
'$builtin_type_descr'(thread_handle, '$union_type'([atom, thread_in_process]), macro_type, "Address formats for threads").
'$builtin_type_descr'(handle, '$union_type'([agent_handle, thread_handle]), macro_type, "All the different address formats of a thread for communication").
'$builtin_type_descr'(process_handle, '$constr_enum_type'([@(atom, atom)]), defined_type, "").
'$builtin_type_descr'(thread_in_process, '$constr_enum_type'([:(atom, agent_handle)]), defined_type, "Address format for communication to another thread of an\nagent or QuLog process on same host").
'$builtin_type_descr'(this_process_name, this_process_name(:('$$var$$'('Name'), ?(atom))), rel, "The Pedro registered name of this process is Name.").
'$builtin_type_descr'(this_task_name, this_task_name(?(atom)), rel, "Unifies it's argument with the name of this task.").
'$builtin_type_descr'(this_agent_name, this_agent_name(?(atom)), rel, "Unifies it's argument with the name of this agent.").
'$builtin_type_descr'(this_thread_name, this_thread_name(:('$$var$$'('Name'), ?(atom))), rel, "The name of thread executing the call is Name.").
'$builtin_type_descr'(connect_to_pedro, connect_to_pedro(default(:('$$var$$'('Host'), !(atom)), localhost), default(:('$$var$$'('Port'), !(int)), 4550)), act, "Connect to the pedro server on Host using Port.").
'$builtin_type_descr'(register_with_pedro, register_with_pedro(!(atom)), act, "Register process name with the Pedro server.\nDeregister and reregister if already connected.").
'$builtin_type_descr'(deregister_from_pedro, deregister_from_pedro('$none_'), act, "Deregister process name with the Pedro server").
'$builtin_type_descr'(disconnect_from_pedro, disconnect_from_pedro('$none_'), act, "Disconnect from the Pedro server").
'$builtin_type_descr'(subscribe, subscribe(!(string), ?(nat)), act, "Use the first arg as a string representing a Pedro subscription\nand subscribe using that. The second argument is instantiated to \nthe Id of the subscription.").
'$builtin_type_descr'(unsubscribe, unsubscribe(:('$$var$$'('ID'), !(nat))), act, "If ID is the Pedro subscription of this process then that \nsubscription will be removed.").
'$builtin_type_descr'(same_thread_handle, same_thread_handle(!(handle), ?(handle)), rel, "").
'$builtin_type_descr'(same_agent_handle, same_agent_handle(!(agent_handle), ?(agent_handle)), rel, "").
'$builtin_type_descr'(send_robotic_message, send_robotic_message(!(robot_message)), act, "").
'$builtin_type_descr'(set_default_message_thread, set_default_message_thread(!(atom)), act, "Changes the destination of messages sent to this agent/process without\na thread identification, to the thread named by the atom arg.").
'$builtin_type_descr'(to, to(:('$$var$$'('Msg'), ??(term)), :('$$var$$'('Handle'), !(agent_handle))), act, "Message send action. Sends Msg to the  agent \ngiven by Handle.").
'$builtin_type_descr'(to_thread, to_thread(??(term), !(thread_handle)), act, "").
'$builtin_type_descr'(peek_messages, peek_messages(:('$$var$$'('Msg'), ??(term)), :('$$var$$'('Addr'), ?(thread_handle)), default(:('$$var$$'('Timeout'), !(int)), -1)), rel, "Search message buffer for Msg from Addr and fail if no match is found\nwithin Timeout seconds (-1 means to block until message arrives)").
'$builtin_type_descr'(true, true('$none_'), rel, "Relation call that always immediately succeeds.").
'$builtin_type_descr'('$true', '$true'('$none_'), rel, "").
'$builtin_type_descr'(false, false('$none_'), rel, "Relation call that always immediately fails.").
'$builtin_type_descr'(remote_query_t_, '$constr_enum_type'([remote_query(string)]), defined_type, "Type for remote queries - for internal use").
'$builtin_type_descr'(respond_remote_query, respond_remote_query(!(string), !(thread_handle)), act, "Takes a query string from a client, parses it, type checks it and then calls \nit - answer bindings, together with the query ID, are returned to the client").
'$builtin_type_descr'(remote_query_call, remote_query_call(!(handle)), rel, "A dummy declaration to be used with allowed_remote_query_from to \nallow remote queries from withing remote queries").
'$builtin_type_descr'(call, call(:('$$var$$'('C'), ??(rel_term))), rel, "Call the relation term C. The type checker checks that C is callable as a\nrelation - i.e. it has a ground functor and the arguments have suitable types \nand modes.").
'$builtin_type_descr'(do, do(:('$$var$$'('A'), ??(act_term))), act, "Call the action term A. The type checker checks that A is callable as an\naction - i.e. it has a ground functor and the arguments have suitable types \nand modes.").
'$builtin_type_descr'(exec_time, ->(exec_time('$none_'), num), fun, "Returns lapsed time in secs since qulog process was forked.").
'$builtin_type_descr'(now, ->(now('$none_'), num), fun, "Returns Unix epoch time as a number of seconds").
'$builtin_type_descr'(start_time, ->(start_time('$none_'), num), fun, "Returns now() value when the qulog process was started.").
'$builtin_type_descr'(time_, time_(?(num)), rel, "Dynamic belief maintained by default agent's percept and message handlers.\nIt always records the time of the most recent belief store update.").
'$builtin_type_descr'(remember_for, remember_for(list(dyn_term), num), act, "").
'$builtin_type_descr'($, ->($(rel('$tuple_type'([int]))), int), fun, "Operator that returns numeric  value linked with an atom name").
'$builtin_type_descr'($, ->($(rel('$tuple_type'([num]))), num), fun, "Operator that returns numeric  value linked with an atom name").
'$builtin_type_descr'(+:=, +:=(!(rel('$tuple_type'([?(num)]))), !(num)), act, "Increment number associated with LHS atom by RHS value").
'$builtin_type_descr'(+:=, +:=(!(rel('$tuple_type'([?(int)]))), !(int)), act, "Increment number associated with LHS atom by RHS value").
'$builtin_type_descr'(-:=, -:=(!(rel('$tuple_type'([?(num)]))), !(num)), act, "Decrement number associated with LHS atom by RHS value").
'$builtin_type_descr'(-:=, -:=(!(rel('$tuple_type'([?(int)]))), !(int)), act, "Decrement number associated with LHS atom by RHS value").
'$builtin_type_descr'(:=, :=(!(rel('$tuple_type'([?(num)]))), !(num)), act, "Change number associated with LHS atom by RHS value").
'$builtin_type_descr'(:=, :=(!(rel('$tuple_type'([?(int)]))), !(int)), act, "Change number associated with LHS atom by RHS value").
'$builtin_type_descr'(hash_table_insert, hash_table_insert(:('$$var$$'('Table'), !(atom)), :('$$var$$'('Key'), !(atomic)), :('$$var$$'('Value'), !(term))), act, "Insert the key-value pair Key, Value into the hash table with name Table").
'$builtin_type_descr'(hash_table_remove, hash_table_remove(:('$$var$$'('Table'), !(atom)), :('$$var$$'('Key'), !(atomic))), act, "Remove the pair with key Key from the hash table Table").
'$builtin_type_descr'(hash_table_lookup, hash_table_lookup(:('$$var$$'('Table'), !(atom)), :('$$var$$'('Key'), !(atomic)), :('$$var$$'('Value'), ?(term))), rel, "Lookup the value associated with the key Key in the hash table \nwith name Table").
'$builtin_type_descr'(hash_table_search, hash_table_search(:('$$var$$'('Table'), ?(atom)), :('$$var$$'('Key'), ?(atomic)), :('$$var$$'('Value'), ?(term))), rel, "Search the hash tables for entries that match Table, Key, Value").
'$builtin_type_descr'(between, between(:('$$var$$'('Start'), !(int)), :('$$var$$'('End'), !(int)), :('$$var$$'('N'), ?(int))), rel, "Test or generate N such that Start =< N =< End").
'$builtin_type_descr'(range, range(:('$$var$$'('N'), ?(int)), :('$$var$$'('Start'), !(int)), :('$$var$$'('End'), !(int)), default(:('$$var$$'('Step'), !(int)), 1)), rel, "Similar to Python range - Test or generate N such that \nStart =< N < End and N = Start + k*Step for some K in nat").
'$builtin_type_descr'(hand_shake_, '$enum_type'([initialise_, finalise_]), defined_type, "Type of the pair of atom messages sent by a TeleoR agents to\na robotic interface to initiate and terminate control").
'$builtin_type_descr'(robot_message, '$union_type'([hand_shake_, list(tel_action_term)]), macro_type, "Union of hand_shake_ and robotic actions").
'$builtin_type_descr'(handle_robotic_actions, handle_robotic_actions('$none_'), act, "User defined action for embedded agent to translate robotic actions").
'$builtin_type_descr'(copy_term, copy_term(:('$$var$$'('Term'), ??(term)), :('$$var$$'('Copy'), ??(term))), rel, "Copy Term with all variables in Term replaced by fresh variables.\nThe first argument is left unchanged.").
'$builtin_type_descr'(poll_sensors, poll_sensors(?(list(tel_percept_term))), act, "User defined action for embedded agent percept collection").
'$builtin_type_descr'(post_process_percepts, post_process_percepts(!(list(tel_percept_term)), !(list(tel_percept_term))), act, "User defined action for post processing percepts").
'$builtin_type_descr'(qmain, qmain(!(list(string))), act, "Entry point for qulog runtime system - same as main for prolog").
'$builtin_type_descr'(resources_hook, resources_hook('$none_'), act, "").
'$builtin_type_descr'(handle_message, handle_message(!(message), !(agent_handle)), act, "System declared, user defined handler for messages sent to a TR agent").
'$builtin_type_descr'(handle_invalid_message, handle_invalid_message(!(string), !(agent_handle)), act, "System declared, user defined handler for invalid messages sent to a TR agent").
'$builtin_type_descr'(handle_template_message, handle_template_message(??(message), !(agent_handle)), act, "System declared, user defined handler for template messages sent to a TR agent").
'$builtin_type_descr'(init_message_handler, init_message_handler('$none_'), act, "").
'$builtin_type_descr'(init_percept_handler, init_percept_handler('$none_'), act, "").
'$builtin_type_descr'(received_message, received_message(term, agent_handle, num), dyn, "Used by the default message handler to store recieved messages.").
'$builtin_type_descr'(start_agent, start_agent(!(agent_handle), atom), act, "Command that can be used only in teleor extension of qulog. \nFirst arg. is the handle of the robot interface or simulation with which \nthe agent will interact, and the second argument\nis the percepts update convention being used: all or updates.").
'$builtin_type_descr'(start_embedded_agent, start_embedded_agent(!(num)), act, "Command that can be used only in teleor extension of qulog. \nThe argument is the reactivity of the interface thread i.e. the \ntime between percept collection events.").
'$builtin_type_descr'(start_task, start_task(:('$$var$$'('TR'), !(tel_term)), :('$$var$$'('Name'), ?(atom)), default(:('$$var$$'('Root'), !(atom)), task)), act, "Start the TR procedure and give it the task name Name based on Root by \nextending Root with 1, 2, ... until an unused task name is found.").
'$builtin_type_descr'(start_named_task, start_named_task(:('$$var$$'('TR'), !(tel_term)), :('$$var$$'('Name'), !(atom))), act, "Start the TR procedure and give it the task name Name.").
'$builtin_type_descr'(kill_task, kill_task(!(atom)), act, "").
'$builtin_type_descr'(kill_agent, kill_agent('$none_'), act, "").
'$builtin_type_descr'(actions, actions(!(robot_message)), act, "Interpreter command that can be used in teleor mode when an agent\nhas been started using start_agent. It is followed by a list\nof actions that agent wants the robotic interface to perform. \nThe given actions must have been robotic actions.").
'$builtin_type_descr'(refresh_bs, refresh_bs('$none_'), act, "Used to enter a list of beliefs that replace the existing beliefs. \nUseful for debugging teleor programs as the user can check which TR procedure \nguards are true for a given state of the belief store.").
'$builtin_type_descr'(task, task(?(atom), ?(tel_term)), rel, "Dynamic relation remembered by start_task and forgotten by kill_task.\nAtom arg. is task name, term arg is term denoting the TeleoR procedure\n call it is executing.").
'$builtin_type_descr'(running, running(:('$$var$$'('Task'), ?(atom))), rel, "True if Task is a running task").
'$builtin_type_descr'(waiting, waiting(:('$$var$$'('Task'), ?(atom))), rel, "True if Task is a waiting task").
'$builtin_type_descr'(resources, resources(:('$$var$$'('Task'), ?(atom)), :('$$var$$'('Resources'), ?(list(resource)))), rel, "True if Task is a running task using Resources resources").
'$builtin_type_descr'(resource_info_, '$constr_enum_type'([res_(atom, list(resource))]), defined_type, "").
'$builtin_type_descr'(get_active_resources, get_active_resources(?(list(resource_info_))), rel, "Get the resources being used by the current running tasks as a list\nof  resource_info_ terms where each term is of the form \nres_(TaskName, ResourceList) where each resource in ResourceList is\neither a declared resource or all__ when no resources have been declared.").
'$builtin_type_descr'(get_waiting_resources, get_waiting_resources(?(list(resource_info_))), rel, "Get the resources needed by the current waiting tasks as a list\nof  resource_info_ terms where each term is of the form \nres_(TaskName, ResourceList) where each resource in ResourceList is\neither a declared resource or all__ when no resources have been declared.").
'$builtin_type_descr'(overlapping_resources, overlapping_resources(!(list(resource)), !(list(resource))), rel, "").
'$builtin_type_descr'(control_device, control_device(!(list(!(tel_action_term))), !(list(tel_action_term))), act, "User defined - required for embedded agents (only).\nThe arguments are the old actions and the new actions.").
'$builtin_type_descr'('$dynamic', '$union_type'(['$belief', '$percept']), macro_type, "").
'$builtin_type_info'(stream_num_type_, defined_type, stream_num_type_, '$constr_enum_type'([stream_(nat)]), []).
'$builtin_type_info'(stream_enum_type_, defined_type, stream_enum_type_, '$enum_type'([stdin, stdout, stderr]), []).
'$builtin_type_info'(stream_type, macro_type, stream_type, '$union_type'([stream_enum_type_, stream_num_type_]), []).
'$builtin_type_info'(open, act, [!(atom), !(atom), ?(stream_type)], open(_12939, _12941, _12943), [=(_12952, 'FileName'), =(_12961, 'RW'), =(_12970, 'Stream')]).
'$builtin_type_info'(close, act, [!(stream_type)], close(_12925), [=(_12934, 'Stream')]).
'$builtin_type_info'(write_type_nl__, defined_type, write_type_nl__, '$enum_type'([nl_]), []).
'$builtin_type_info'(write_type_other__, defined_type, write_type_other__, '$constr_enum_type'([sp_(int), q_(string), uq_(atom), wr_(term)]), []).
'$builtin_type_info'(write_type__, macro_type, write_type__, '$union_type'([write_type_nl__, write_type_other__]), []).
'$builtin_type_info'(get_line, act, [?(string), !(stream_type)], get_line(_12932, _12934), [=(_12943, 'Line'), =(_12952, 'Stream')]).
'$builtin_type_info'(put_line, act, [!(string), !(stream_type)], put_line(_12932, _12934), [=(_12943, 'Line'), =(_12952, 'Stream')]).
'$builtin_type_info'(read_term, act, [??(term), !(stream_type)], read_term(_12932, _12934), [=(_12943, 'Stream')]).
'$builtin_type_info'(read_op_term, act, [??(term), !(stream_type)], read_op_term(_12932, _12934), [=(_12943, 'Stream')]).
'$builtin_type_info'(peek, act, [?(atom), !(stream_type)], peek(_12932, _12934), [=(_12943, 'Stream')]).
'$builtin_type_info'(flush_output, act, [!(stream_type)], flush_output(_12925), [=(_12934, 'Stream')]).
'$builtin_type_info'(term2string, rel, [??(term), ?(string)], term2string(_12932, _12934), [=(_12943, 'Term'), =(_12952, 'String')]).
'$builtin_type_info'(str, fun, ->('$tuple_type'([term]), string), '$function', []).
'$builtin_type_info'(string2term, rel, [!(string), ??(term)], string2term(_12932, _12934), [=(_12943, 'String'), =(_12952, 'Term')]).
'$builtin_type_info'(write_list, act, [!(list(??(term))), !(stream_type)], write_list(_12938, _12940), [=(_12949, 'TermList'), =(_12958, 'Stream')]).
'$builtin_type_info'(write_list_to_string, rel, [!(list(??(term))), ?(string)], write_list_to_string(_12938, _12940), [=(_12949, 'TermList'), =(_12958, 'String')]).
'$builtin_type_info'(write_op_term, act, [??(term), !(stream_type)], write_op_term(_12932, _12934), [=(_12943, 'Term'), =(_12952, 'Stream')]).
'$builtin_type_info'(log_list, act, [!(list(??(term)))], log_list(_12931), []).
'$builtin_type_info'(writeDebug, rel, [!(list(??(term)))], writeDebug(_12931), []).
'$builtin_type_info'(*, fun, ->('$tuple_type'([nat, nat]), nat), '$function', []).
'$builtin_type_info'(*, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(*, fun, ->('$tuple_type'([num, num]), num), '$function', []).
'$builtin_type_info'(**, fun, ->('$tuple_type'([nat, nat]), nat), '$function', []).
'$builtin_type_info'(**, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(**, fun, ->('$tuple_type'([num, num]), num), '$function', []).
'$builtin_type_info'(+, fun, ->('$tuple_type'([nat, nat]), nat), '$function', []).
'$builtin_type_info'(+, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(+, fun, ->('$tuple_type'([num, num]), num), '$function', []).
'$builtin_type_info'(-, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(-, fun, ->('$tuple_type'([num, num]), num), '$function', []).
'$builtin_type_info'(/, fun, ->('$tuple_type'([num, num]), num), '$function', []).
'$builtin_type_info'(//, fun, ->('$tuple_type'([nat, nat]), nat), '$function', []).
'$builtin_type_info'(//, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(/\, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(>>, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(\, fun, ->('$tuple_type'([int]), int), '$function', []).
'$builtin_type_info'(\/, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(abs, fun, ->('$tuple_type'([int]), nat), '$function', []).
'$builtin_type_info'(abs, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(acos, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(asin, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(atan, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(atan2, fun, ->('$tuple_type'([num, num]), num), '$function', [=(_12936, 'Y'), =(_12945, 'X')]).
'$builtin_type_info'(ceiling, fun, ->('$tuple_type'([num]), int), '$function', []).
'$builtin_type_info'(cos, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(e, fun, ->('$tuple_type'([]), num), '$function', []).
'$builtin_type_info'(floor, fun, ->('$tuple_type'([num]), int), '$function', []).
'$builtin_type_info'(log, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(max, fun, ->('$tuple_type'([nat, nat]), nat), '$function', []).
'$builtin_type_info'(max, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(max, fun, ->('$tuple_type'([num, num]), num), '$function', []).
'$builtin_type_info'(min, fun, ->('$tuple_type'([nat, nat]), nat), '$function', []).
'$builtin_type_info'(min, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(min, fun, ->('$tuple_type'([num, num]), num), '$function', []).
'$builtin_type_info'(mod, fun, ->('$tuple_type'([int, int]), int), '$function', [=(_12936, 'M'), =(_12945, 'N')]).
'$builtin_type_info'(pi, fun, ->('$tuple_type'([]), num), '$function', []).
'$builtin_type_info'(prod, fun, ->('$tuple_type'([list(int)]), int), '$function', []).
'$builtin_type_info'(prod, fun, ->('$tuple_type'([list(num)]), num), '$function', []).
'$builtin_type_info'(round, fun, ->('$tuple_type'([num]), int), '$function', []).
'$builtin_type_info'(sin, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(sqrt, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(random_int, fun, ->('$tuple_type'([int, int]), int), '$function', [=(_12936, 'M'), =(_12945, 'N')]).
'$builtin_type_info'(random_num, fun, ->('$tuple_type'([]), num), '$function', []).
'$builtin_type_info'(random_seed, act, [?(nat)], random_seed(_12925), [=(_12934, 'S')]).
'$builtin_type_info'(sum, fun, ->('$tuple_type'([list(int)]), int), '$function', []).
'$builtin_type_info'(sum, fun, ->('$tuple_type'([list(num)]), num), '$function', []).
'$builtin_type_info'(tan, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(<<, fun, ->('$tuple_type'([int, int]), int), '$function', []).
'$builtin_type_info'(-, fun, ->('$tuple_type'([int]), int), '$function', []).
'$builtin_type_info'(-, fun, ->('$tuple_type'([num]), num), '$function', []).
'$builtin_type_info'(=, rel, [??(term), ??(term)], =(_12932, _12934), []).
'$builtin_type_info'(=@, rel, [?(_12922), !(_12922)], =@(_12934, _12936), [=(_12922, 'T')]).
'$builtin_type_info'(=@, rel, [??(_12922), ??(_12922)], =@(_12934, _12936), [=(_12922, 'T')]).
'$builtin_type_info'(<, rel, [!(num), !(num)], <(_12932, _12934), []).
'$builtin_type_info'(=<, rel, [!(num), !(num)], =<(_12932, _12934), []).
'$builtin_type_info'(>, rel, [!(num), !(num)], >(_12932, _12934), []).
'$builtin_type_info'(>=, rel, [!(num), !(num)], >=(_12932, _12934), []).
'$builtin_type_info'(@<, rel, [??(term), ??(term)], @<(_12932, _12934), []).
'$builtin_type_info'(@=<, rel, [??(term), ??(term)], @=<(_12932, _12934), []).
'$builtin_type_info'(@>, rel, [??(term), ??(term)], @>(_12932, _12934), []).
'$builtin_type_info'(@>=, rel, [??(term), ??(term)], @>=(_12932, _12934), []).
'$builtin_type_info'(\=, rel, [??(term), ??(term)], \=(_12932, _12934), []).
'$builtin_type_info'(isa, rel, [?(term), !(typeE(!(_12934)))], isa(_12940, _12942), []).
'$builtin_type_info'(template, rel, [??(term)], template(_12925), []).
'$builtin_type_info'(var, rel, [??(term)], var(_12925), []).
'$builtin_type_info'(type, rel, [??(term), !(typeE(!(_12934)))], type(_12940, _12942), [=(_12934, 'T')]).
'$builtin_type_info'(allowed_dyn_call, rel, [??(term)], allowed_dyn_call(_12925), []).
'$builtin_type_info'(allowed_rel_call, rel, [??(term)], allowed_rel_call(_12925), []).
'$builtin_type_info'(allowed_act_call, rel, [??(term)], allowed_act_call(_12925), []).
'$builtin_type_info'(type_kind_, defined_type, type_kind_, '$constr_enum_type'([enumT_(list(atomic)), constrT_(list(term)), unionT_(list(term)), tupleT_(list(term)), rangeT_(int, int)]), []).
'$builtin_type_info'(type_info, rel, [!(typeE(!(_12928))), ?(type_kind_)], type_info(_12940, _12942), [=(_12928, 'T')]).
'$builtin_type_info'(#, fun, ->('$tuple_type'([list(term)]), nat), '$function', []).
'$builtin_type_info'(#, fun, ->('$tuple_type'([set(term)]), nat), '$function', []).
'$builtin_type_info'(#, fun, ->('$tuple_type'([string]), nat), '$function', []).
'$builtin_type_info'(in, rel, [?(_12922), !(list(!(_12922)))], in(_12940, _12942), [=(_12922, 'T')]).
'$builtin_type_info'(in, rel, [??(_12922), !(list(??(_12922)))], in(_12940, _12942), [=(_12922, 'T')]).
'$builtin_type_info'(in, rel, [?(_12922), !(set(!(_12922)))], in(_12940, _12942), [=(_12922, 'T')]).
'$builtin_type_info'(in, rel, [?(string), !(string)], in(_12932, _12934), [=(_12943, 'T')]).
'$builtin_type_info'(tolist, fun, ->('$tuple_type'([string]), list(string)), '$function', [=(_12936, 'T')]).
'$builtin_type_info'(tolist, fun, ->('$tuple_type'([set(_12929)]), list(_12929)), '$function', [=(_12929, 'T')]).
'$builtin_type_info'(toset, fun, ->('$tuple_type'([string]), set(string)), '$function', [=(_12936, 'T')]).
'$builtin_type_info'(toset, fun, ->('$tuple_type'([list(_12929)]), set(_12929)), '$function', [=(_12929, 'T')]).
'$builtin_type_info'(<>, fun, ->('$tuple_type'([list(_12929), list(_12929)]), list(_12929)), '$function', [=(_12929, 'T')]).
'$builtin_type_info'(append, rel, [!(list(!(_12928))), !(list(!(_12928))), ?(list(?(_12928)))], append(_12959, _12961, _12963), [=(_12928, 'T')]).
'$builtin_type_info'(append, rel, [?(list(?(_12928))), ?(list(?(_12928))), !(list(!(_12928)))], append(_12959, _12961, _12963), [=(_12928, 'T')]).
'$builtin_type_info'(append, rel, [!(list(??(_12928))), !(list(??(_12928))), ?(list(??(_12928)))], append(_12959, _12961, _12963), [=(_12928, 'T')]).
'$builtin_type_info'(append, rel, [?(list(??(_12928))), ?(list(??(_12928))), !(list(??(_12928)))], append(_12959, _12961, _12963), [=(_12928, 'T')]).
'$builtin_type_info'(append, rel, [?(list(??(_12928))), ??(list(??(_12928))), ??(list(??(_12928)))], append(_12959, _12961, _12963), [=(_12928, 'T')]).
'$builtin_type_info'(delete, rel, [?(_12922), !(list(!(_12922))), ?(list(?(_12922)))], delete(_12953, _12955, _12957), [=(_12966, 'X'), =(_12922, 'T'), =(_12982, 'Xs'), =(_12991, 'R')]).
'$builtin_type_info'(member, rel, [?(_12922), !(list(!(_12922)))], member(_12940, _12942), [=(_12922, 'T')]).
'$builtin_type_info'(member, rel, [??(_12922), ??(list(??(_12922)))], member(_12940, _12942), [=(_12922, 'T')]).
'$builtin_type_info'(reverse, rel, [!(list(!(_12928))), ?(list(?(_12928)))], reverse(_12946, _12948), [=(_12928, 'T')]).
'$builtin_type_info'(reverse, rel, [?(list(?(_12928))), !(list(!(_12928)))], reverse(_12946, _12948), [=(_12928, 'T')]).
'$builtin_type_info'(reverse, rel, [!(list(??(_12928))), ?(list(??(_12928)))], reverse(_12946, _12948), [=(_12928, 'T')]).
'$builtin_type_info'(reverse, rel, [?(list(??(_12928))), !(list(??(_12928)))], reverse(_12946, _12948), [=(_12928, 'T')]).
'$builtin_type_info'(sort, rel, [!(list(!(_12928))), ?(list(?(_12928))), !(rel('$tuple_type'([!(_12928), !(_12928)])))], sort(_12971, _12973, _12975), [=(_12928, 'T')]).
'$builtin_type_info'(sort, rel, [!(list(??(_12928))), ?(list(??(_12928))), !(rel('$tuple_type'([??(_12928), ??(_12928)])))], sort(_12971, _12973, _12975), [=(_12928, 'T')]).
'$builtin_type_info'(transform_subterms, rel, [!(rel('$tuple_type'([??(term), ??(term)]))), ??(term), ??(term)], transform_subterms(_12957, _12959, _12961), []).
'$builtin_type_info'(collect_simple_terms, rel, [!(rel('$tuple_type'([??(term), ??(term), ??(term)]))), ??(term), ??(term), ??(term)], collect_simple_terms(_12970, _12972, _12974, _12976), []).
'$builtin_type_info'(term2list, rel, [!(term), ?(list(?(term)))], term2list(_12938, _12940), []).
'$builtin_type_info'(term2list, rel, [?(term), !(list(!(term)))], term2list(_12938, _12940), []).
'$builtin_type_info'(term2list, rel, [??(term), ??(list(??(term)))], term2list(_12938, _12940), []).
'$builtin_type_info'(diff, fun, ->('$tuple_type'([set(_12929), set(_12929)]), set(_12929)), '$function', [=(_12929, 'T')]).
'$builtin_type_info'(inter, fun, ->('$tuple_type'([set(_12929), set(_12929)]), set(_12929)), '$function', [=(_12929, 'T')]).
'$builtin_type_info'(union, fun, ->('$tuple_type'([set(_12929), set(_12929)]), set(_12929)), '$function', [=(_12929, 'T')]).
'$builtin_type_info'(++, fun, ->('$tuple_type'([string, string]), string), '$function', []).
'$builtin_type_info'(sub_string, rel, [!(string), ?(nat), ?(int), ?(nat), ?(string)], sub_string(_12953, _12955, _12957, _12959, _12961), [=(_12970, 'String'), =(_12979, 'Start'), =(_12988, 'Length'), =(_12997, 'After'), =(_13006, 'SubString')]).
'$builtin_type_info'(re_match, rel, [!(string), !(string), ?(list(?('$tuple_type'([?(nat), ?(nat)]))))], re_match(_12960, _12962, _12964), [=(_12973, 'RE'), =(_12982, 'String'), =(_12991, 'Match')]).
'$builtin_type_info'(map, fun, ->('$tuple_type'([->('$tuple_type'([_12938]), _12930), list(_12938)]), list(_12930)), '$function', [=(_12956, 'F'), =(_12938, 'T1'), =(_12930, 'T2'), =(_12979, 'List')]).
'$builtin_type_info'(filter, fun, ->('$tuple_type'([rel('$tuple_type'([_12935])), list(_12935)]), list(_12935)), '$function', [=(_12953, 'Test'), =(_12935, 'T1'), =(_12969, 'List')]).
'$builtin_type_info'(filter_map, fun, ->('$tuple_type'([rel('$tuple_type'([_12935])), ->('$tuple_type'([_12935]), _12944), list(_12935)]), list(_12944)), '$function', [=(_12968, 'Test'), =(_12935, 'T1'), =(_12984, 'F'), =(_12944, 'T2'), =(_13000, 'List')]).
'$builtin_type_info'(builtin_exception, defined_type, builtin_exception, '$constr_enum_type'([read_term_syntax_error('$none_'), input_term_type_error(atom, string), read_op_term_syntax_error(string), input_term_error(atom, string), string2term_syntax_error(string), stream_not_writeable(string), stream_not_readable(string), stream_not_open(string), prolog_call_type_error(string), action_failure(string), no_matching_action_rule(string), no_matching_function_rule(string), no_matching_tr_rule(tel_term), arithmetic_function_domain_error(term), remote_query_ontology_mismatch(string), remote_query_exception(exception), pedro_not_connected(atom), pedro_name_already_registered(atom), fork_thread_name_exists(atom), qp_exception(string), qulog2qp_type_exception(string), no_agent_to_kill('$none_'), no_matching_task_to_kill(atom), not_start_task_TR_procedure(string), task_name_in_use(atom)]), []).
'$builtin_type_info'(exception, macro_type, exception, '$union_type'([builtin_exception, user_exception]), []).
'$builtin_type_info'(raise, act, [!(exception)], raise(_12925), [=(_12934, 'Patt')]).
'$builtin_type_info'(thread_sizes_t, defined_type, thread_sizes_t, '$constr_enum_type'([choice_size(nat), env_size(nat), heap_size(nat), binding_trail_size(nat), other_trail_size(nat), scratchpad_size(nat), name_table_size(nat), ip_table_size(nat)]), []).
'$builtin_type_info'(exit, act, [], exit('$none_'), []).
'$builtin_type_info'(exit_thread, act, [!(atom)], exit_thread(_12925), []).
'$builtin_type_info'(fork, act, [!(act_term), ?(atom), !(list(!(thread_sizes_t)))], fork(_12945, _12947, _12949), [=(_12958, 'Act'), =(_12967, 'Name'), =(_12976, 'Sizes')]).
'$builtin_type_info'(fork_light, act, [!(act_term), ?(atom)], fork_light(_12932, _12934), [=(_12943, 'Act'), =(_12952, 'Name')]).
'$builtin_type_info'(thread_sleep, act, [!(num)], thread_sleep(_12925), []).
'$builtin_type_info'(is_thread, rel, [!(atom)], is_thread(_12925), [=(_12934, 'Th')]).
'$builtin_type_info'(agent_handle, macro_type, agent_handle, '$union_type'([atom, process_handle]), []).
'$builtin_type_info'(thread_handle, macro_type, thread_handle, '$union_type'([atom, thread_in_process]), []).
'$builtin_type_info'(handle, macro_type, handle, '$union_type'([agent_handle, thread_handle]), []).
'$builtin_type_info'(process_handle, defined_type, process_handle, '$constr_enum_type'([@(atom, atom)]), []).
'$builtin_type_info'(thread_in_process, defined_type, thread_in_process, '$constr_enum_type'([:(atom, agent_handle)]), []).
'$builtin_type_info'(this_process_name, rel, [?(atom)], this_process_name(_12925), [=(_12934, 'Name')]).
'$builtin_type_info'(this_task_name, rel, [?(atom)], this_task_name(_12925), []).
'$builtin_type_info'(this_agent_name, rel, [?(atom)], this_agent_name(_12925), []).
'$builtin_type_info'(this_thread_name, rel, [?(atom)], this_thread_name(_12925), [=(_12934, 'Name')]).
'$builtin_type_info'(connect_to_pedro, act, [!(atom), !(int)], connect_to_pedro(_12932, _12934), [=(_12943, 'Host'), =(_12952, 'Port')]).
'$builtin_type_info'(register_with_pedro, act, [!(atom)], register_with_pedro(_12925), []).
'$builtin_type_info'(deregister_from_pedro, act, [], deregister_from_pedro('$none_'), []).
'$builtin_type_info'(disconnect_from_pedro, act, [], disconnect_from_pedro('$none_'), []).
'$builtin_type_info'(subscribe, act, [!(string), ?(nat)], subscribe(_12932, _12934), []).
'$builtin_type_info'(unsubscribe, act, [!(nat)], unsubscribe(_12925), [=(_12934, 'ID')]).
'$builtin_type_info'(same_thread_handle, rel, [!(handle), ?(handle)], same_thread_handle(_12932, _12934), []).
'$builtin_type_info'(same_agent_handle, rel, [!(agent_handle), ?(agent_handle)], same_agent_handle(_12932, _12934), []).
'$builtin_type_info'(send_robotic_message, act, [!(robot_message)], send_robotic_message(_12925), []).
'$builtin_type_info'(set_default_message_thread, act, [!(atom)], set_default_message_thread(_12925), []).
'$builtin_type_info'(to, act, [??(term), !(agent_handle)], to(_12932, _12934), [=(_12943, 'Msg'), =(_12952, 'Handle')]).
'$builtin_type_info'(to_thread, act, [??(term), !(thread_handle)], to_thread(_12932, _12934), []).
'$builtin_type_info'(peek_messages, rel, [??(term), ?(thread_handle), !(int)], peek_messages(_12939, _12941, _12943), [=(_12952, 'Msg'), =(_12961, 'Addr'), =(_12970, 'Timeout')]).
'$builtin_type_info'(true, rel, [], true('$none_'), []).
'$builtin_type_info'('$true', rel, [], '$true'('$none_'), []).
'$builtin_type_info'(false, rel, [], false('$none_'), []).
'$builtin_type_info'(remote_query_t_, defined_type, remote_query_t_, '$constr_enum_type'([remote_query(string)]), []).
'$builtin_type_info'(respond_remote_query, act, [!(string), !(thread_handle)], respond_remote_query(_12932, _12934), []).
'$builtin_type_info'(remote_query_call, rel, [!(handle)], remote_query_call(_12925), []).
'$builtin_type_info'(call, rel, [??(rel_term)], call(_12925), [=(_12934, 'C')]).
'$builtin_type_info'(do, act, [??(act_term)], do(_12925), [=(_12934, 'A')]).
'$builtin_type_info'(exec_time, fun, ->('$tuple_type'([]), num), '$function', []).
'$builtin_type_info'(now, fun, ->('$tuple_type'([]), num), '$function', []).
'$builtin_type_info'(start_time, fun, ->('$tuple_type'([]), num), '$function', []).
'$builtin_type_info'(time_, rel, [?(num)], time_(_12925), []).
'$builtin_type_info'(remember_for, act, [!(list(!(dyn_term))), !(num)], remember_for(_12938, _12940), []).
'$builtin_type_info'($, fun, ->('$tuple_type'([rel('$tuple_type'([int]))]), int), '$function', []).
'$builtin_type_info'($, fun, ->('$tuple_type'([rel('$tuple_type'([num]))]), num), '$function', []).
'$builtin_type_info'(+:=, act, [!(rel('$tuple_type'([?(num)]))), !(num)], +:=(_12944, _12946), []).
'$builtin_type_info'(+:=, act, [!(rel('$tuple_type'([?(int)]))), !(int)], +:=(_12944, _12946), []).
'$builtin_type_info'(-:=, act, [!(rel('$tuple_type'([?(num)]))), !(num)], -:=(_12944, _12946), []).
'$builtin_type_info'(-:=, act, [!(rel('$tuple_type'([?(int)]))), !(int)], -:=(_12944, _12946), []).
'$builtin_type_info'(:=, act, [!(rel('$tuple_type'([?(num)]))), !(num)], :=(_12944, _12946), []).
'$builtin_type_info'(:=, act, [!(rel('$tuple_type'([?(int)]))), !(int)], :=(_12944, _12946), []).
'$builtin_type_info'(hash_table_insert, act, [!(atom), !(atomic), !(term)], hash_table_insert(_12939, _12941, _12943), [=(_12952, 'Table'), =(_12961, 'Key'), =(_12970, 'Value')]).
'$builtin_type_info'(hash_table_remove, act, [!(atom), !(atomic)], hash_table_remove(_12932, _12934), [=(_12943, 'Table'), =(_12952, 'Key')]).
'$builtin_type_info'(hash_table_lookup, rel, [!(atom), !(atomic), ?(term)], hash_table_lookup(_12939, _12941, _12943), [=(_12952, 'Table'), =(_12961, 'Key'), =(_12970, 'Value')]).
'$builtin_type_info'(hash_table_search, rel, [?(atom), ?(atomic), ?(term)], hash_table_search(_12939, _12941, _12943), [=(_12952, 'Table'), =(_12961, 'Key'), =(_12970, 'Value')]).
'$builtin_type_info'(between, rel, [!(int), !(int), ?(int)], between(_12939, _12941, _12943), [=(_12952, 'Start'), =(_12961, 'End'), =(_12970, 'N')]).
'$builtin_type_info'(range, rel, [?(int), !(int), !(int), !(int)], range(_12946, _12948, _12950, _12952), [=(_12961, 'N'), =(_12970, 'Start'), =(_12979, 'End'), =(_12988, 'Step')]).
'$builtin_type_info'(hand_shake_, defined_type, hand_shake_, '$enum_type'([initialise_, finalise_]), []).
'$builtin_type_info'(robot_message, macro_type, robot_message, '$union_type'([hand_shake_, list(tel_action_term)]), []).
'$builtin_type_info'(handle_robotic_actions, act, [], handle_robotic_actions('$none_'), []).
'$builtin_type_info'(copy_term, rel, [??(term), ??(term)], copy_term(_12932, _12934), [=(_12943, 'Term'), =(_12952, 'Copy')]).
'$builtin_type_info'(poll_sensors, act, [?(list(?(tel_percept_term)))], poll_sensors(_12931), []).
'$builtin_type_info'(post_process_percepts, act, [!(list(!(tel_percept_term))), !(list(!(tel_percept_term)))], post_process_percepts(_12944, _12946), []).
'$builtin_type_info'(qmain, act, [!(list(!(string)))], qmain(_12931), []).
:-('$builtin_type_info'(resources_hook, act, [], resources_hook('$none_'), []), '$running_teleo').
:-('$builtin_type_info'(handle_message, act, [!(message), !(agent_handle)], handle_message(_12932, _12934), []), '$running_teleo').
:-('$builtin_type_info'(handle_invalid_message, act, [!(string), !(agent_handle)], handle_invalid_message(_12932, _12934), []), '$running_teleo').
:-('$builtin_type_info'(handle_template_message, act, [??(message), !(agent_handle)], handle_template_message(_12932, _12934), []), '$running_teleo').
:-('$builtin_type_info'(init_message_handler, act, [], init_message_handler('$none_'), []), '$running_teleo').
:-('$builtin_type_info'(init_percept_handler, act, [], init_percept_handler('$none_'), []), '$running_teleo').
:-('$builtin_type_info'(received_message, rel, [?(term), ?(agent_handle), ?(num)], received_message(_12939, _12941, _12943), []), '$running_teleo').
:-('$builtin_type_info'(start_agent, act, [!(agent_handle), !(atom)], start_agent(_12932, _12934), []), '$running_teleo').
:-('$builtin_type_info'(start_embedded_agent, act, [!(num)], start_embedded_agent(_12925), []), '$running_teleo').
:-('$builtin_type_info'(start_task, act, [!(tel_term), ?(atom), !(atom)], start_task(_12939, _12941, _12943), [=(_12952, 'TR'), =(_12961, 'Name'), =(_12970, 'Root')]), '$running_teleo').
'$builtin_type_info'(start_named_task, act, [!(tel_term), !(atom)], start_named_task(_12932, _12934), [=(_12943, 'TR'), =(_12952, 'Name')]).
:-('$builtin_type_info'(kill_task, act, [!(atom)], kill_task(_12925), []), '$running_teleo').
:-('$builtin_type_info'(kill_agent, act, [], kill_agent('$none_'), []), '$running_teleo').
:-('$builtin_type_info'(actions, act, [!(robot_message)], actions(_12925), []), '$running_teleo').
:-('$builtin_type_info'(refresh_bs, act, [], refresh_bs('$none_'), []), '$running_teleo').
:-('$builtin_type_info'(task, rel, [?(atom), ?(tel_term)], task(_12932, _12934), []), '$running_teleo').
:-('$builtin_type_info'(running, rel, [?(atom)], running(_12925), [=(_12934, 'Task')]), '$running_teleo').
:-('$builtin_type_info'(waiting, rel, [?(atom)], waiting(_12925), [=(_12934, 'Task')]), '$running_teleo').
:-('$builtin_type_info'(resources, rel, [?(atom), ?(list(?(resource)))], resources(_12938, _12940), [=(_12949, 'Task'), =(_12958, 'Resources')]), '$running_teleo').
'$builtin_type_info'(resource_info_, defined_type, resource_info_, '$constr_enum_type'([res_(atom, list(resource))]), []).
:-('$builtin_type_info'(get_active_resources, rel, [?(list(?(resource_info_)))], get_active_resources(_12931), []), '$running_teleo').
:-('$builtin_type_info'(get_waiting_resources, rel, [?(list(?(resource_info_)))], get_waiting_resources(_12931), []), '$running_teleo').
'$builtin_type_info'(overlapping_resources, rel, [!(list(!(resource))), !(list(!(resource)))], overlapping_resources(_12944, _12946), []).
'$builtin_type_info'(control_device, act, [!(list(!(tel_action_term))), !(list(!(tel_action_term)))], control_device(_12944, _12946), []).
'$builtin_type_info'('$dynamic', macro_type, '$dynamic', '$union_type'(['$belief', '$percept']), []).
:-('$builtin_type_info'(user_exception, defined_type, user_exception, '$constr_enum_type'([default_exception_('$none_')]), []), \+('$user_type_info'(user_exception, _12927, _12929, _12931, _12933))).
'#$apply'(#, [A], B) :- 
    #(A, B).
'$$apply'($, [A], B) :- 
    $(A, B).
'*$apply'(*, [A, B], C) :- 
    *(A, B, C).
'**$apply'(**, [A, B], C) :- 
    **(A, B, C).
'+$apply'(+, [A, B], C) :- 
    +(A, B, C).
'++$apply'(++, [A, B], C) :- 
    string_concat(A, B, C).
'/$apply'(/, [A, B], C) :- 
    /(A, B, C).
'//$apply'(//, [A, B], C) :- 
    //(A, B, C).
'/\\$apply'(/\, [A, B], C) :- 
    /\(A, B, C).
'<<$apply'(<<, [A, B], C) :- 
    <<(A, B, C).
'<>$apply'(<>, [A, B], C) :- 
    append(A, B, C).
'>>$apply'(>>, [A, B], C) :- 
    >>(A, B, C).
'\\$apply'(\, [A], B) :- 
    \(A, B).
'\\/$apply'(\/, [A, B], C) :- 
    \/(A, B, C).
'abs$apply'(abs, [A], B) :- 
    abs(A, B).
'acos$apply'(acos, [A], B) :- 
    acos(A, B).
'asin$apply'(asin, [A], B) :- 
    asin(A, B).
'atan$apply'(atan, [A], B) :- 
    atan(A, B).
'atan2$apply'(atan2, [A, B], C) :- 
    atan2(A, B, C).
'ceiling$apply'(ceiling, [A], B) :- 
    ceiling(A, B).
'cos$apply'(cos, [A], B) :- 
    cos(A, B).
'diff$apply'(diff, [A, B], C) :- 
    diff(A, B, C).
'e$apply'(e, [], A) :- 
    e(A).
'exec_time$apply'(exec_time, [], A) :- 
    exec_time(A).
'filter$apply'(filter, [A, B], C) :- 
    filter(A, B, C).
'filter_map$apply'(filter_map, [A, B, C], D) :- 
    filter_map(A, B, C, D).
'floor$apply'(floor, [A], B) :- 
    floor(A, B).
'inter$apply'(inter, [A, B], C) :- 
    inter(A, B, C).
'log$apply'(log, [A], B) :- 
    log(A, B).
'map$apply'(map, [A, B], C) :- 
    map(A, B, C).
'max$apply'(max, [A, B], C) :- 
    max(A, B, C).
'min$apply'(min, [A, B], C) :- 
    min(A, B, C).
'mod$apply'(mod, [A, B], C) :- 
    mod(A, B, C).
'now$apply'(now, [], A) :- 
    now(A).
'pi$apply'(pi, [], A) :- 
    pi(A).
'prod$apply'(prod, [A], B) :- 
    prod(A, B).
'random_int$apply'(random_int, [A, B], C) :- 
    random_int(A, B, C).
'random_num$apply'(random_num, [], A) :- 
    random_num(A).
'round$apply'(round, [A], B) :- 
    round(A, B).
'sin$apply'(sin, [A], B) :- 
    sin(A, B).
'sqrt$apply'(sqrt, [A], B) :- 
    sqrt(A, B).
'start_time$apply'(start_time, [], A) :- 
    start_time(A).
'str$apply'(str, [A], B) :- 
    str(A, B).
'sum$apply'(sum, [A], B) :- 
    sum(A, B).
'tan$apply'(tan, [A], B) :- 
    tan(A, B).
'tolist$apply'(tolist, [A], B) :- 
    tolist(A, B).
'toset$apply'(toset, [A], B) :- 
    toset(A, B).
'union$apply'(union, [A, B], C) :- 
    union(A, B, C).
'-$apply'(-, (A, B), C) :- 
    !,
    -(A, B, C).
'-$apply'(-, A, B) :- 
    -(A, B).
