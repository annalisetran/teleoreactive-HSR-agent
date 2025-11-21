#include "QuProlog.h"
extern "C" long twice(long);
extern  void triple(double, double*);
extern  void mkfoo(char **);

extern "C" bool
twice_interface(ForeignInterface* fi)
{
 bool result = true;
 long integer0;
 long integer1;
 Object* object0;
 Object* object1;
 Object* outarg1;

 object0 = fi->getXReg(0);
 if (!object0->isInteger())
 {
  return(false);
 }
 integer0 = object0->getInteger();

 object1 = fi->getXReg(1);

 integer1 = twice(integer0);
 outarg1 = fi->makeInteger(integer1);
 result = result && fi->unify(object1, outarg1);

 return(result);
}

extern "C" bool
triple_interface(ForeignInterface* fi)
{
 bool result = true;
 double float0;
 double float1;
 Object* object0;
 Object* object1;
 Object* outarg1;

 object0 = fi->getXReg(0);
 if (!object0->isDouble())
 {
  return(false);
 }
 if (object0->isInteger())
 {
  float0 = object0->getInteger();
 }
 else
 {
  float0 = object0->getDouble();
 }
 object1 = fi->getXReg(1);


 triple(float0, &float1);
 outarg1 = fi->makeDouble(float1);
 result = result && fi->unify(object1, outarg1);

 return(result);
}

extern "C" bool
mkfoo_interface(ForeignInterface* fi)
{
 bool result = true;
 char * atom0;
 Object* object0;
 Object* outarg0;

 object0 = fi->getXReg(0);


 mkfoo(&atom0);
 outarg0 = fi->makeAtom(atom0);
 result = result && fi->unify(object0, outarg0);

 return(result);
}
