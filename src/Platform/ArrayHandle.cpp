/*
 * ArrayHandle.cpp
 *
 *  Created on: 12 Jun 2022
 *      Author: David
 */

#include "ArrayHandle.h"
#include <ObjectModel/ObjectModel.h>

// Declare placement new operator for class ExpressionValue so that we don't have to include <new>
void* operator new(std::size_t, ExpressionValue* arg) noexcept { return arg; }

struct ArrayStorageSpace
{
	uint16_t length;								// length of this object in bytes including this length field, always rounded up to a multiple of 4
	uint16_t count;									// number of elements in the array
	ExpressionValue elements[];						// the array elements
};

// Allocate space for an array
void ArrayHandle::Allocate(size_t numElements) THROWS(GCodeException)
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasWriteLock();
#endif
	Heap::IndexSlot * const slot = Heap::AllocateHandle();
	Heap::StorageSpace * const space = Heap::AllocateSpace(sizeof(ArrayStorageSpace) + numElements * sizeof(ExpressionValue));
	slot->storage = space;
	if (space->length < sizeof(ArrayStorageSpace) + numElements * sizeof(ExpressionValue))
	{
		Heap::DeleteSlot(slot);
		throw GCodeException("Array too large");
	}

	ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(space);
	aSpace->count = numElements;
	for (size_t i = 0; i < numElements; ++i)
	{
		new (&aSpace->elements[i]) ExpressionValue;
	}
	slotPtr = slot;
}

// Assign an element without making the array unique first
void ArrayHandle::AssignElement(size_t index, ExpressionValue &val) THROWS(GCodeException)
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasWriteLock();
#endif
	if (slotPtr != nullptr)
	{
		ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
		if (index < aSpace->count)
		{
			aSpace->elements[index] = val;
			return;
		}
	}
	throw GCodeException("array index out of bounds");
}

// Make an array unique and assign an element, possibly nested
void ArrayHandle::AssignIndexed(const ExpressionValue& ev, size_t numIndices, const uint32_t *indices) THROWS(GCodeException)
{
	WriteLocker locker(Heap::heapLock);							// prevent other tasks modifying the heap
	// We may need to allocate heap space to complete this assignment. However this may result
	// in garbage collection moving some of the storage used for the array, which may invalidate
	// pointers to that storage. To avoid this we restart the assigment process (and so re calculate any
	// pointers) if we have needed to allocate storage. The previously alocated storage is then provided
	// via the newStorage variable.
	ArrayHandle newStorage;
	while (!InternalAssignIndexed(ev, numIndices, indices, newStorage))
	{
	}
}

// Make an array unique and assign an element, possibly nested
// return true if the assigment completed, false if we had to allocate new stoarge and so need to be
// called again to complete the operation
bool ArrayHandle::InternalAssignIndexed(const ExpressionValue& ev, size_t numIndices, const uint32_t *indices, ArrayHandle& newStorage) THROWS(GCodeException)
{
	if (indices[0] >= GetNumElements())
	{
		throw GCodeException("array index out of bounds");
	}
	if (!MakeUnique(newStorage))
	{
		// we needed to allocate new storage, so unwind the stack and start again
		return false;
	}
	ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
	if (numIndices == 1)
	{
		aSpace->elements[indices[0]] = ev;
	}
	else if (aSpace->elements[indices[0]].GetType() != TypeCode::HeapArray)
	{
		throw GCodeException("attempt to index into a non-array");
	}
	else
	{
		return aSpace->elements[indices[0]].ahVal.InternalAssignIndexed(ev, numIndices - 1, indices + 1, newStorage);
	}
	return true;
}

// Get the number of elements in a heap array
// Caller must have a read lock or a write lock on heapLock before calling this!
size_t ArrayHandle::GetNumElements() const noexcept
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasReadOrWriteLock();
#endif
	if (slotPtr == nullptr)
	{
		return 0;
	}
#if CHECK_HANDLES
	Heap::CheckSlotGood(slotPtr);
#endif
	return reinterpret_cast<const ArrayStorageSpace*>(slotPtr->storage)->count;
}

// Retrieve an array element, returning false if the index is out of bounds
// Caller must have a read lock on heapLock before calling this!
bool ArrayHandle::GetElement(size_t index, ExpressionValue &rslt) const noexcept
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasReadLock();
#endif
	if (slotPtr != nullptr)
	{
#if CHECK_HANDLES
		Heap::CheckSlotGood(slotPtr);
#endif
		ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
		if (index < aSpace->count)
		{
			rslt = aSpace->elements[index];
			return true;
		}
	}
	return false;
}

// Retrieve the type of an array element, or TypeCode::None if the index is out of range
// Caller must have a read lock on heapLock before calling this!
TypeCode ArrayHandle::GetElementType(size_t index) const noexcept
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasReadLock();
#endif
	if (slotPtr != nullptr)
	{
#if CHECK_HANDLES
		Heap::CheckSlotGood(slotPtr);
#endif
		ArrayStorageSpace * const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
		if (index < aSpace->count)
		{
			return aSpace->elements[index].GetType();
		}
	}
	return TypeCode::None;
}

// Decrease the reference count for the array referred to and delete it if it reaches zero
void ArrayHandle::Delete() noexcept
{
	if (slotPtr != nullptr)
	{
		ReadLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
		if (slotPtr->refCount == 1)
		{
			// The call to Heap:::DeleteSlot will deallocate the slot, so release the contents first
			ArrayStorageSpace *const aSpace = reinterpret_cast<ArrayStorageSpace*>(slotPtr->storage);
			for (size_t i = 0; i < aSpace->count; ++i)
			{
				aSpace->elements[i].~ExpressionValue();			// call destructor on the elements
			}
		}
		Heap::DeleteSlot(slotPtr);
		slotPtr = nullptr;										// clear the pointer to the handle entry
	}
}

const ArrayHandle& ArrayHandle::IncreaseRefCount() const noexcept
{
	if (slotPtr != nullptr)
	{
		Heap::IncreaseRefCount(slotPtr);
	}
	return *this;
}

// Make this handle refer to non-shared array (the individual elements may be shared). Caller must already own a read lock on the heap.
// return true if we completed the operation, false if we need to be called again
bool ArrayHandle::MakeUnique(ArrayHandle& ah2) THROWS(GCodeException)
{
#if CHECK_HEAP_LOCKED
	Heap::heapLock.CheckHasWriteLock();
#endif
	if (slotPtr != nullptr && slotPtr->refCount > 1)
	{
		const ArrayStorageSpace * const aSpace = reinterpret_cast<const ArrayStorageSpace*>(slotPtr->storage);
		const size_t count = aSpace->count;
		// Have we previously allocated the needed storage?
		if (ah2.slotPtr == nullptr)
		{
			// no so allocate the storage and return so we can be called again
			ah2.Allocate(count);
			return false;
		}
		ArrayStorageSpace * const aSpace2 = reinterpret_cast<ArrayStorageSpace*>(ah2.slotPtr->storage);
		for (size_t i = 0; i < count; ++i)
		{
			aSpace2->elements[i] = aSpace->elements[i];
		}

		--slotPtr->refCount;
		slotPtr = ah2.slotPtr;
		ah2.slotPtr = nullptr;
	}
	return true;
}

// AutoArrayHandle members

AutoArrayHandle::AutoArrayHandle(const AutoArrayHandle &other) noexcept
	: ArrayHandle(other)
{
	IncreaseRefCount();
}

AutoArrayHandle::AutoArrayHandle(AutoArrayHandle &&other) noexcept
	: ArrayHandle(other)
{
	other.slotPtr = nullptr;
}

AutoArrayHandle& AutoArrayHandle::operator =(const AutoArrayHandle &other) noexcept
{
	if (slotPtr != other.slotPtr)
	{
		Delete();
		slotPtr = other.slotPtr;
		IncreaseRefCount();
	}
	return *this;
}

AutoArrayHandle::~AutoArrayHandle()
{
	ArrayHandle::Delete();
}

// End
