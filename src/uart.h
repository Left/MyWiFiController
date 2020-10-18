#pragma once

#include <stddef.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

#ifdef STM32
extern HardwareSerial debugSerial;
#endif

namespace uart {
    #pragma pack(push, 1)
    struct Signature {
        uint8_t start0 = 42;
        uint8_t start1 = 19;
        uint8_t start2 = 53;
        uint8_t start3 = 11;
        uint32_t size = 0;
    };
    #pragma pack(pop)

    Signature def = {};

    struct Sender {
        typedef std::function<size_t(const uint8_t* buffer, size_t size)> SenderFunc;

        Sender(SenderFunc fn) : sender(fn) {}

        void send(const uint8_t* msg, size_t sz) {
            Signature sig;
            sig.size = sz;
            // Send size
            sendImpl((const uint8_t*)&sig, sizeof(sig));
            // Send buffer
            sendImpl(msg, sz);
        }

    private:
        void sendImpl(const uint8_t* buffer, size_t size) {
            for (size_t sz = size; sz > 0;) {
                sz -= sender(buffer + (size-sz), sz);
            }
        }

        SenderFunc sender;
    };

    struct Receiver {
        typedef std::function<size_t(void)> AvailFunc;
        typedef std::function<size_t(uint8_t* buffer, size_t size)> ReceiveFunc;

        Receiver(AvailFunc avail_, ReceiveFunc rcvr_) : avail(avail_), rcvr(rcvr_) {}

        void receive(std::function<void(uint8_t* buffer, size_t size)> processor) {
            size_t av = avail();
            if (av == 0) {
                return;
            }

            size_t old_size = buffer.size();
            buffer.resize(old_size + av);
            size_t read = rcvr(&buffer[old_size], av);
            buffer.resize(old_size + read);

            // Attempt to process messages
            uint8_t* bufStart = &*buffer.begin();
            uint8_t *bufEnd = &*buffer.end();

            auto it = std::search(
                bufStart, bufEnd,
                &def.start0, &def.start3 + 1);
            if (it != bufEnd) {
                // OK, header is here

                if (bufEnd - it >= sizeof(Signature)) {
                    // OK, the whole signature is here
                    auto sigPtr = (Signature*) &*it;
                    auto sz = sigPtr->size;

                    if (bufEnd - it >= (sz + sizeof(Signature))) {
                        // OK, the whole msg is here
                        auto msgPtr = it + sizeof(Signature);

                        processor(msgPtr, sz);

                        bufStart = it + sizeof(Signature) + sz; // Go to next msg
                    }
                }
            } 

            if (bufStart != &*buffer.begin()) {
                memcpy(&*buffer.begin(), bufStart, bufEnd - bufStart);
                buffer.resize(bufEnd - bufStart);
            }
        }

    private:
        AvailFunc avail;
        ReceiveFunc rcvr;

        std::vector<uint8_t> buffer;
    };
}