package buffers;

import java.util.ArrayList;

public class QueuedBuffer<T> implements Buffer<T> {

	protected ArrayList<T> payload = new ArrayList<T>();

	public QueuedBuffer() {

	}

	public void push(T payload) {
		this.payload.add(payload);

	}

	public T getNext() {
		return this.payload.size() > 0 ? this.payload.remove(0) : null;
	}
}
