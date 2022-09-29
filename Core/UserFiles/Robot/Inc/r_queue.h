#ifndef R_QUEUE_H_
#define R_QUEUE_H_

#ifdef __cplusplus
/*
https://stackoverflow.com/questions/70954201/trying-to-incorporate-a-c-file-in-cubeide
 */

template <typename Element> class R_Queue
{
public:
  R_Queue(int alen);
  ~R_Queue();
  bool push(Element elem);
  Element pop();
  bool isFull() const;
  bool isEmpty() const;
  int getFreeSpace() const;
  int getMaxLength() const;
  inline int getUsedSpace() const;
private:
  R_Queue(R_Queue<Element>& q);  //copy const.
  Element* data;
  int len;
  int start;
  int count;
};

template <typename Element> R_Queue<Element>::R_Queue(int alen)
{
  data = new Element[alen];
  len = alen;
  start = 0;
  count = 0;
}

template <typename Element> R_Queue<Element>::~R_Queue()
{
  delete data;
}

template <typename Element> R_Queue<Element>::R_Queue(R_Queue<Element>& q)
{
  //nothing ever is allowed to do something here
}

template <typename Element> bool R_Queue<Element>::push(Element elem)
{
  data[(start + count++) % len] = elem;
}

template <typename Element> Element R_Queue<Element>::pop()
{
  count--;
  int s = start;
  start = (start + 1) % len;
  return data[(s) % len];
}

template <typename Element> bool R_Queue<Element>::isFull() const
{
  return count >= len;
}

template <typename Element>
bool R_Queue<Element>::isEmpty() const {
  return count <= 0;
}

template <typename Element> int R_Queue<Element>::getFreeSpace() const
{
  return len - count;
}

template <typename Element> int R_Queue<Element>::getMaxLength() const
{
  return len;
}

template <typename Element> int R_Queue<Element>::getUsedSpace() const
{
  return count;
}

#endif

#endif
