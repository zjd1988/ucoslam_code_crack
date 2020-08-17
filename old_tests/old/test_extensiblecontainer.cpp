#include "stuff/expansiblecontainer.h"
#include <fstream>
template<typename T>
struct StreamableVar{

    StreamableVar(){}
    StreamableVar(T val){var=val;}
    T var;
    void toStream (std::ostream &str)const{str.write((char*)&var,sizeof(T));}
    void fromStream (std::istream &str){str.read((char*)&var,sizeof(T));}
};

int f(const ucoslam::ExpansibleContainer<StreamableVar<int>> &C){
   return C.at(12).var;
}


int main(){


    ucoslam::ExpansibleContainer<StreamableVar<int> > Exp;
    for(int i=0;i<100000;i++)
        Exp.push_back(i);
    f(Exp);

    std::ofstream file("aux.bin",std::ios::binary);
    Exp.toStream(file);
    file.close();
    ucoslam::ExpansibleContainer<StreamableVar<int> > Exp2;
    std::ifstream fileIn("aux.bin",std::ios::binary);
    Exp2.fromStream(fileIn);
    fileIn.close();
    for(int i=0;i<100000;i++)
        assert(Exp2[i].var==i);
    std::ofstream file3("aux2.bin",std::ios::binary);
    Exp2.toStream(file3);
    file3.close();

}
