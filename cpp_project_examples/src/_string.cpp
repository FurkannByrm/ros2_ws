#include <iostream>


int main(){

    std::string string1 = "an unnecessary sentence";
    if(string1.empty() == 0)//emty string içinde bir veri varsa 0 yoksa 1 değeri dondurur
    {
      std::cout<<string1.at(2)<<std::endl; //3. indisteki kararkter gösteren ifade
      std::cout<<string1.length()<<std::endl; // string uzunluğunu sayar boşluklarala birlikte
      std::cout<<string1<<std::endl;
      std::cout<<string1.find("den")<<std::endl; // string içinde kelime aramamıza yarar
      // cıkan veri bize o kelime yada harfin başladığı indisi verir. eğer çıkan sayı 
      //çok yüksekse bu aradığımız kelimenin string içinde olmadığını gösterir 

      std::cout<<string1.find_last_of("den")<<std::endl;// bu sefer sondan başa arama yapılır
      string1.insert(0,"bu",3);//stringin istediğimiz kısmına ekleme yapmak
      //için kullanılır ilk parametre başlangıcı ikincisi eklenecek kısmı 3. ise uzunluk


      string1.erase(0,3);// silmek istediğimiz aralığı yazarız
      std::cout<<string1<<std::endl;
      string1.clear();// stingin icinde bulunan veriyi siler.
      std::cout<<string1.empty()<<std::endl;//bize 1 ciktisini veriri yane ici bos oldugunu gosterir
    }

    std::string str = "Ankara";
    string1 = str.substr(2,3);//str icinden secilen kismi string1 yerlestirir.
    std::cout<<string1;

    return 0;

    
}