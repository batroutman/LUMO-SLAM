package buffers;

public interface Buffer<T> {

	public void push(T payload);

	public T getNext();

}
