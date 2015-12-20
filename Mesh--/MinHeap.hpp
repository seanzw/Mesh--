#ifndef MIN_HEAP_HEADER
#define MIN_HEAP_HEADER

template <typename T>
class MinHeap {
public:

    MinHeap(int capacity) : capacity(capacity), size(0) {
        buf = new T[capacity];
    }

    ~MinHeap() {
        if (capacity > 0) {
            delete[] buf;
        }
    }

    T pop(int id = 0) {
        if (id < = 0 || id >= size) {
            throw "Index out of range. ";
        }
        else {
            T ret = buf[id];
            swap(id, size - 1);
            size--;
            restore(id);
            return ret;
        }
    }

    void push(const T &t) {
        assert(size < capacity);
        buf[size] = t;
        size++;
        restore(size - 1);
    }

private:
    int capacity;
    int size;
    T *buf;

    void swap(int i, int j) {
        T tmp = buf[i];
        buf[i] = buf[j];
        buf[j] = tmp;
    }

    void restore(int id) {

        auto ups = [&](int id)->void {
            while (id > 0) {
                int parent = (id - 1) / 2;
                if (buf[parent] < buf[id]) {
                    break;
                }
                else {
                    swap(id, parent);
                    id = parent;
                }
            }
        };

        auto dwn = [&](int id)->void {
            while (true) {
                int lhs = id * 2 + 1;
                int rhs = id * 2 + 2;
                if (rhs < size) {
                    if (buf[lhs] < buf[rhs] && buf[lhs] < buf[id]) {
                        swap(lhs, id);
                        id = lhs;
                    }
                    else if (buf[rhs] < buf[lhs] && buf[rhs] < buf[id]) {
                        swap(rhs, id);
                        id = rhs;
                    }
                    else {
                        break;
                    }
                }
                else if (lhs < size) {
                    if (buf[lhs] < buf[id]) {
                        swap(lhs, id);
                    }
                    break;
                }
                else {
                    break;
                }
            }
        }

        ups(id);
        dwn(id);
    }

};

#endif