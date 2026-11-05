#pragma once
#include <iostream>
#include <type_traits>
using namespace std;

template <typename T> class priQueue
{
  private:
    struct Node
    {
        T data;
        int priority;
        Node *next;
        Node(T d, int p) : data(d), priority(p), next(nullptr) {}
    };
    Node *head;
    int count;

  public:
    priQueue() : head(nullptr), count(0) {}
    priQueue(const priQueue &other) : head(nullptr), count(0)
    {
        Node *cur = other.head;
        while (cur)
        {
            enqueue(cur->data, cur->priority);
            cur = cur->next;
        }
    }
    ~priQueue() { clear(); }

    priQueue &operator=(const priQueue &other)
    {
        if (this == &other)
            return *this;
        clear();
        Node *cur = other.head;
        while (cur)
        {
            enqueue(cur->data, cur->priority);
            cur = cur->next;
        }
        return *this;
    }

    bool isEmpty() const { return count == 0; }
    int getCount() const { return count; }

    void enqueue(T item, int priority)
    {
        Node *node = new Node(item, priority);
        if (isEmpty() || priority > head->priority)
        {
            node->next = head;
            head = node;
        }
        else
        {
            Node *cur = head;
            while (cur->next && cur->next->priority >= priority)
                cur = cur->next;
            node->next = cur->next;
            cur->next = node;
        }
        count++;
    }

    // Failure recovery must put the patient ahead of every waiting request.
    void enqueueFront(T item, int priority)
    {
        Node *node = new Node(item, priority);
        node->next = head;
        head = node;
        ++count;
    }

    bool dequeue(T &item, int &priority)
    {
        if (isEmpty())
            return false;
        Node *temp = head;
        item = head->data;
        priority = head->priority;
        head = head->next;
        delete temp;
        count--;
        return true;
    }

    bool peek(T &item, int &priority) const
    {
        if (isEmpty())
            return false;
        item = head->data;
        priority = head->priority;
        return true;
    }

    void clear()
    {
        Node *cur = head;
        while (cur)
        {
            Node *nxt = cur->next;
            delete cur;
            cur = nxt;
        }
        head = nullptr;
        count = 0;
    }

    void print() const
    {
        Node *cur = head;
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
