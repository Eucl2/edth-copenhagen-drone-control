use crate::drone::Point;
use drone::{DroneClientMsg, DroneServerMsg};
use eyre::Result;
use nalgebra::Vector3;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{metadata::MetadataValue, transport::Channel, Request};

pub mod drone {
    tonic::include_proto!("drone");
}

#[tokio::main]
async fn main() -> Result<()> {
    // TODO: add your token here
    let token = "Bearer my_token";
    println!("{:?}", token);

    let channel = Channel::from_static("http://172.104.137.51:10301")
        .connect()
        .await?;
    let token: MetadataValue<_> = token.parse()?;
    let mut client = drone::drone_controller_client::DroneControllerClient::with_interceptor(
        channel,
        move |mut req: Request<()>| {
            req.metadata_mut().insert("authorization", token.clone());
            Ok(req)
        },
    );

    let (req_tx, req_rx) = mpsc::channel(128);
    let request_stream = ReceiverStream::new(req_rx);
    let mut response_stream: tonic::Streaming<DroneServerMsg> =
        client.drone_connection(request_stream).await?.into_inner();

    let first_res = response_stream
        .message()
        .await
        .unwrap()
        .unwrap()
        .data
        .unwrap();

    let drone::drone_server_msg::Data::Start(starting_info) = first_res else {
        panic!("first message was not a sim start, failing");
    };

    let mut locations: Vec<Location> = Vec::new();
    locations.push(starting_info.drone_location.unwrap().into());
    let goal = starting_info.goal.unwrap();
    let minimal_goal: Vector3<f32> = goal.minimal_point.unwrap().into();
    let maximal_goal: Vector3<f32> = goal.maximal_point.unwrap().into();
    let target = (minimal_goal + maximal_goal).scale(0.5);
    println!("target set: {:?}", target);

    loop {
        let response = response_stream
            .message()
            .await
            .unwrap()
            .unwrap()
            .data
            .unwrap();

        match response {
            drone::drone_server_msg::Data::Start(_) => {
                panic!("shouldn't receive start message now")
            }
            drone::drone_server_msg::Data::Ended(over) => {
                println!(
                    "Simulation over! Success = {}, details = {:?}",
                    over.success, over.details
                );
                return Ok(());
            }
            drone::drone_server_msg::Data::Update(update) => {
                let point = update.drone_location.unwrap();
                println!("location: {:?}", point);
            }
        };

        let msg = DroneClientMsg {
            throttle: 75,
            roll: 0,
            pitch: 0,
        };

        println!("control request: {:?}", msg);
        req_tx.send(msg).await?;
    }
}

type Location = Vector3<f32>;

impl From<Point> for Vector3<f32> {
    fn from(value: Point) -> Self {
        Vector3::new(value.x, value.y, value.z)
    }
}
