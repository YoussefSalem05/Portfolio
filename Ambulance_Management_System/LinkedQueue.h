#pragma once
#include <iostream>
#include <type_traits>
using namespace std;

template <typename T> class LinkedQueue
{
  private:
    struct Node
    {
        T data;
        Node *next;
        Node(T d) : data(d), next(nullptr) {}
    };
    Node *front;
    Node *rear;
    int count;

  public:
    LinkedQueue() : front(nullptr), rear(nullptr), count(0) {}
    LinkedQueue(const LinkedQueue &other) : front(nullptr), rear(nullptr), count(0)
    {
        Node *cur = other.front;
        while (cur)
        {
            enqueue(cur->data);
            cur = cur->next;
        }
    }
    ~LinkedQueue() { clear(); }

    LinkedQueue &operator=(const LinkedQueue &other)
    {
        if (this == &other)
            return *this;
        clear();
        Node *cur = other.front;
        while (cur)
        {
            enqueue(cur->data);
            cur = cur->next;
        }
        return *this;
    }

    bool isEmpty() const { return count == 0; }
    int getCount() const { return count; }

    void enqueue(T item)
    {
        Node *node = new Node(item);
        if (isEmpty())
            front = rear = node;
        else
        {
            rear->next = node;
            rear = node;
        }
        count++;
    }

    void enqueueFront(T item)
    {
        Node *node = new Node(item);
        if (isEmpty())
        {
            front = rear = node;
        }
        else
        {
            node->next = front;
            front = node;
        }
        count++;
    }

    bool dequeue(T &item)
    {
        if (isEmpty())
            return false;
        Node *temp = front;
        item = front->data;
        front = front->next;
        if (front == nullptr)
            rear = nullptr;
        delete temp;
        count--;
        return true;
    }

    bool peek(T &item) const
    {
        if (isEmpty())
            return false;
        item = front->data;
        return true;
    }

    void clear()
    {
        Node *cur = front;
        while (cur)
        {
            Node *nxt = cur->next;
            delete cur;
            cur = nxt;
        }
        front = rear = nullptr;
        count = 0;
    }

    void print() const
    {
        Node *cur = front;
        while (cur)
        {
            if constexpr (std::is_pointer<T>::value)
            {
                if (cur->data)
                    cout << *cur->data << " ";
            }
            else
                cout << cur->data << " ";
            cur = cur->next;
        }
    }
};
