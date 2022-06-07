using System.Collections.Generic;

namespace LinkBake
{
    public class QuickHashList<T>
    {
        private Dictionary<T, int> _dic;
        private List<T> _list;
        private int _size;

        public int Count
        {
            get
            {
                return _size;
            }
        }

        public QuickHashList()
        {
            _dic = new Dictionary<T, int>();
            _list = new List<T>();
            _size = 0;
        }

        public T this[int index]
        {
            get { return _list[index]; }
        }

        public void Add(T value)
        {
            int index;
            if (!_dic.TryGetValue(value, out index))
            {
                if (_size < _list.Count)
                {
                    _list[_size] = value;
                }
                else
                {
                    _list.Add(value);
                }

                ++_size;
                _dic.Add(value, _size - 1);
            }
        }

        public void Remove(T value)
        {
            int index;
            if (_dic.TryGetValue(value, out index))
            {
                _list[index] = _list[_size-1];
                _dic[_list[index]] = index;
                _dic.Remove(value);
                --_size;
            }
        }

        /// <summary>
        /// 查找value在容器中的下标，若没有返回-1
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public int Find(T value)
        {
            int index;
            if (_dic.TryGetValue(value, out index))
            {
                return index;
            }

            return -1;
        }
    }
}
