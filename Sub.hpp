#ifndef SUB_HPP
#define SUB_HPP
/*
  Appunti rigurardo: using namespace std
  Questo comando viene usato per definire nel file lo spazio dei nomi che verrà usato. Esempio:
    std::cout << "Hello world!" << std::endl;
    *Senza la dichiarazione del namespace sono costretto a specificare dove si trova ciò che voglio usare*

    cout << "Hello world!" << endl;
    *Con la dichiarazione del namespace*
*/

#include <rclcpp/rclcpp.hpp> //! rclcpp base library

#include <std_msgs/msg/string.hpp> //! interface library that we'll use

/*Note preliminari: 
Quando ho std --> mi muovo nello spazio di c++
Quando ho rclcpp -> Mi muovo nello spazio di ros
Ogni volta che ho i due punti specifico una specie di percorso path ->
Esempio riga 37 -> &Sub dico che devo muovermi nello spazio di lavoro della funzione Sub e cercare all'interno di Sub callbAck
*/
//Sub è il nome --> Node sono le proprietà che eredita (Sub è un nodo)!
class Sub : public rclcpp::Node{

public:
  
  Sub()  //Costruttore:
  :Node("sub")  //Chiama il costruttore della classe padre passandogli il nome del nodo
  {
    //Instanzio lo shared pointer:
    //Crete_subrisption è un metodo template --> Definisco il percoso del template (specifico che il template sarà costruito su std_msg, che lo sto defininedo che Sub e che leggerà stringhe
    sub_= this->create_subscription<std_msgs::msg::String>(
      "/examples/test_topic", //1° Parametro : Nome topic
      rclcpp::QoS(10),        //Definisco quanti caratteri legge il messaggio dal topic in cui sono iscritto/Profondità buffer
      std::bind(&Sub::callback, /*Puntatore alla funzione stessa*/ this, std::placeholders::_1 /*_1 (callback ha un solo argomento)*/) //Bind è un metodo
    )
}
//! ROS-specific members better be private

private:
  //Il subscriber è un oggetto mantenuto  a parte nel nodo, lo creo in private
  //Subscription è una classe ed ogni Subscription ha una specifica tipologia --> In tal caso usa messaggi di tipo stringa (Sono shared pointer)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; 
  //Callback --> Come primo parametro ho il puntatotore a ciò che ho preso dal topic
  void callback(const std_msgs::msg::String::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str()); //Stampa nel log il contenuto del messaggio appena arrivato (logger del nodo)
  }

  //! Nothing else is necessary to receive messages

};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Sub>(); //auto consente di inferire il tipo della variabile(il compilatore capisce da solo il tipo di nodo)
  rclcpp::spin(node); //esco solo quando termino il nodo, esempio: CTRL+C, ROS gestisce il CTRL+C
  rclcpp::shutdown();
  return 0;
}