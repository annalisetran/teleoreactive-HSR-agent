foreign_file('test.o', [twice, triple, mkfoo]).

foreign(twice, c, twice(+ integer, [- integer])).
foreign(triple, 'c++', triple(+ float, - float)).
foreign(mkfoo, 'c++', mkfoo(- atom)).
