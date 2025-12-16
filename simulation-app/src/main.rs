use actix_web::{get, post, web, App, HttpResponse, HttpServer, Responder, Result};
use actix_files::{Files, NamedFile};


#[actix_web::main]
async fn main() -> std::io::Result<()> {
    HttpServer::new(|| {
        App::new()
            .service(
                Files::new("/", "./www/")
                .index_file("Render.html")
            )
    })
    .bind(("127.0.0.1", 8080))?
    .run().await
}
