#ifndef FIFO_DEF_H_
#define FIFO_DEF_H_

#include <stdint.h>

#include <zephyr/sys/atomic.h>
#include <zephyr/irq.h>


template<size_t LEN,typename CNT=unsigned char>
class FIFO{
	CNT data[LEN];
	CNT* const data_end;
	atomic_ptr_t wrptr;
	atomic_ptr_t rdptr;
public:
	FIFO() : data(), data_end(&data[LEN]), wrptr(data), rdptr(data){};
	void clear()
	{
		atomic_ptr_set(&wrptr, data);
		atomic_ptr_set(&rdptr, data);
	}
	inline intptr_t put(CNT val)
	{
		volatile CNT *tmwr = (volatile CNT *)atomic_ptr_get(&wrptr);
		volatile CNT *tmrd = (volatile CNT *)atomic_ptr_get(&rdptr);

		tmwr += 1;
		if(tmwr == data_end)
			tmwr = data;
		if (tmwr == tmrd)
				return 0;
		*tmwr = val;
		atomic_ptr_set(&wrptr,(void*)tmwr);
		return 1;

	}
	inline intptr_t get(CNT* val)
	{
		volatile CNT *tmwr = (volatile CNT *)atomic_ptr_get(&wrptr);
		volatile CNT *tmrd = (volatile CNT *)atomic_ptr_get(&rdptr);
		if (tmwr == tmrd)
			return 0;
		tmrd += 1;
		if(tmrd == data_end)
			tmrd = data;
		*val = *tmrd;
		atomic_ptr_set(&rdptr,(void*)tmrd);
		return 1;
	}
	inline intptr_t check()
	{
		volatile CNT *tmwr = (volatile CNT *)atomic_ptr_get(&wrptr);
		volatile CNT *tmrd = (volatile CNT *)atomic_ptr_get(&rdptr);
		int ret = (tmwr - tmrd);
		if (ret < 0) ret +=LEN;
		return ret;
	}
};

#endif /* FIFO_H_ */
