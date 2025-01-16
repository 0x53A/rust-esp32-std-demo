use esp_idf_svc::ota::EspOta;
use esp_idf_svc::sys::EspError;
use ring::hmac::{verify, Key};
use mailboxxy_rs::{start_mailbox, MailboxBounds, MailboxContext};

struct OtaChunk {
    offset: u32,
    size: u32,
    data: [u8],
    sha256_hash: [u8]
}

struct OtaHeader {
    total_size: u32,
    sha256_hash: [u8],
    signature_of_hash: [u8]
}


enum OtaMsg {
    Chunk(OtaChunk),
    Commit()
}

pub fn start_ota() {

    async fn ota_actor_fn(ctx: MailboxContext<OtaMsg>) {

        // local state
        let mut ota = EspOta::new()?;
        let mut work = ota.initiate_update()?;

        loop {
            let msg: OtaMsg = ctx.dequeue().await;

            match msg {
                OtaMsg::Chunk(chunk) => {
                    // todo: validate hash of chunk

                    let result = work.write(&chunk.data[..chunk.size]);
                    // todo: validate result
                },
                OtaMsg::Commit => {
                    // todo: validate hash of complete written firmware

                    let result = work.complete();
                    // todo: validate result
                    return
                }
            }
        }
    }

    let mailbox = mailboxxy_rs::start_mailbox(MailboxBounds::Unbounded, ota_actor_fn, task::spawn);

    return mailbox;
}



fn oversimplified_update_firmware(pubKey: Key, header: OtaHeader, chunks: dyn Iterator<OtaChunk>) -> Result<(), EspError> {

    result = verify(public_key, message, signature);

    let mut ota = EspOta::new()?;
    let mut work = ota.initiate_update()?;
    let buff: Vec<u8> = get_firmware_buffer_from_somewhere();
    work.write(&buff)?;
    work.complete()
}

