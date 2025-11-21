//
// foreign(twice, c, twice(+ integer, [- integer])).
//
extern "C" long
twice(long a)
{
    return(2 * a);
}

//
// foreign(triple, c++, triple(+ float, - float)).
//
void
triple(double x, double *a)
{
    *a = (x * 3);
}

//
// foreign(mkfoo, 'c++', mkfoo(- atom)).
//
void mkfoo(char** a)
{
    *a = (char *)"foo";
}
