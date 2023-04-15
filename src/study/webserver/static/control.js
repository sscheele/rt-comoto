// window.setInterval(async() => {
//     const resp = await fetch("/get-which-img")
//     const data = await resp.json()
//     console.log(data)
//     document.getElementById("img").src = "/static/".concat(data["which_img"])
//     window.location.reload()
// })

const intervalID = setInterval(get_posted_img, 1000)

function get_posted_img() {
    // const resp = fetch("/get-which-img")
    // const data = resp.json()
    // console.log(data)
    // document.getElementById("img").src = "/static/".concat(data["which_img"])
    document.location.reload()
}