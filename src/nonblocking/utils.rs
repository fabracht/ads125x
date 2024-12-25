// src/nonblocking/utils.rs

/// Yields control back to the async runtime
pub async fn yield_now() {
    struct Yield {
        yielded: bool,
    }

    impl core::future::Future for Yield {
        type Output = ();

        fn poll(
            mut self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            if !self.yielded {
                self.yielded = true;
                cx.waker().wake_by_ref();
                core::task::Poll::Pending
            } else {
                core::task::Poll::Ready(())
            }
        }
    }

    Yield { yielded: false }.await;
}
