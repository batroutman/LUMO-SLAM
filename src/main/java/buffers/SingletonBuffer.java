package buffers;

import java.util.ArrayList;
import java.util.List;

public class SingletonBuffer<T> implements Buffer<T> {

	protected List<BufferListener<T>> listeners = new ArrayList<BufferListener<T>>();
	protected T payload = null;

	public SingletonBuffer() {

	}

	public void push(T payload) {
		this.payload = payload;
		this.callListeners(payload);
	}

	public T getNext() {
		return this.payload;
	}

	public List<BufferListener<T>> getListeners() {
		return listeners;
	}

	public void setListeners(List<BufferListener<T>> listeners) {
		this.listeners = listeners;
	}

	public void addListener(BufferListener<T> listener) {
		this.listeners.add(listener);
	}

	public void callListeners(T payload) {
		for (BufferListener<T> listener : this.listeners) {
			listener.push(payload);
		}
	}
}
