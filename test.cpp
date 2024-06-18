#include <deque>
#include <iostream>

int main(){

    std::deque<uint8_t> data;
    for(int i = 0; i < 10; i++)
    {
        data.push_back(i);
    }

    data.erase(data.begin(),data.begin() + 6);

    std::cout<<data.size()<<"  "<<data.max_size()<<std::endl;
}