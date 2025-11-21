?- load_foreign(
 [
 'test_interface.o',
 'test.o'
 ],
 [
 twice/2 = twice_interface,
 triple/2 = triple_interface,
 mkfoo/1 = mkfoo_interface
 ],
 [
]).